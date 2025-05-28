#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from math import atan2, sin, cos, degrees, sqrt
from std_msgs.msg import Header, Int32, String, Float32
from xycar_msgs.msg import XycarMotor
from geometry_msgs.msg import PoseStamped, TwistStamped
from numpy.linalg import norm
import rospy
import tf

# 파라미터 설정
TARGET_SPEED = 25  # 목표 속도

class LateralPID:
    """차선 주행용 횡방향 PID 제어기"""
    def __init__(self, kp=1.9, ki=0.03, kd=0.4, dt=0.05):  # 보수적 게인
        self.K_P = kp
        self.K_I = ki
        self.K_D = kd
        self.pre_error = 0.0
        self.integral_error = 0.0
        self.dt = dt
        self.error_history = []  # 에러 히스토리 저장

    def run(self, target_x, current_x=320, image_width=640):
        """
        target_x: 차선 중심 목표 X 좌표 (픽셀)
        current_x: 현재 차량 X 좌표 (이미지 중심)
        """
        # 픽셀 오차 계산
        pixel_error = target_x - current_x
        
        # 에러 히스토리 관리 (이동평균을 위해)
        self.error_history.append(pixel_error)
        if len(self.error_history) > 5:
            self.error_history.pop(0)
        
        # 이동평균으로 노이즈 감소
        smoothed_error = sum(self.error_history) / len(self.error_history)
        
        # 정규화: 더 부드러운 변환
        lateral_error = (smoothed_error / (image_width/4)) * 10.0  # 범위 축소
        
        # 데드존 설정 (작은 오차 무시)
        if abs(lateral_error) < 2.0:
            lateral_error *= 0.5
        
        derivative_error = (lateral_error - self.pre_error) / self.dt
        self.integral_error += lateral_error * self.dt
        self.integral_error = np.clip(self.integral_error, -20, 20)  # 적분 제한
        
        correction = (self.K_P * lateral_error + 
                     self.K_I * self.integral_error + 
                     self.K_D * derivative_error)
        
        self.pre_error = lateral_error
        return np.clip(correction, -20, 20)  # 스케일카 물리적 한계

class SpeedPID:
    """속도 제어용 PID"""
    def __init__(self, kp=1.0, ki=0.05, kd=0.1, dt=0.05):
        self.K_P = kp
        self.K_I = ki
        self.K_D = kd
        self.pre_error = 0.0
        self.integral_error = 0.0
        self.dt = dt

    def run(self, target_speed, current_speed):
        error = target_speed - current_speed
        derivative_error = (error - self.pre_error) / self.dt
        self.integral_error += error * self.dt
        self.integral_error = np.clip(self.integral_error, -5, 5)
        
        throttle = (self.K_P * error + 
                   self.K_I * self.integral_error + 
                   self.K_D * derivative_error)
        
        self.pre_error = error
        return np.clip(throttle, 0, 40)

class LaneDrivingController:
    def __init__(self, shared, steer_ratio=1.0):
        self.shared = shared
        self.ego = shared.ego if hasattr(shared, 'ego') else None
        self.perception = shared.perception
        
        self.lateral_pid = LateralPID(kp=1.9, ki=0.03, kd=0.4)
        self.speed_pid = SpeedPID(kp=1.5, ki=0.05, kd=0.2)
        self.steer_ratio = steer_ratio
        
        self.prev_steer = 0.0
        self.steer_history = []
        
        self.target_x = 320
        self.lane_status = "BOTH"
        self.lane_curvature = 0.0
        
        self.target_x_received = False
        self.lane_status_received = False
        
        rospy.Subscriber('/lane_detection/target_x', Int32, self.target_x_callback)
        rospy.Subscriber('/lane_detection/lane_status', String, self.lane_status_callback)
        rospy.Subscriber('/lane_detection/lane_curvature', Float32, self.lane_curvature_callback)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_callback)
        
        rospy.loginfo("Stable Lane Driving Controller initialized")

    def target_x_callback(self, msg):
        if 50 <= msg.data <= 590:
            self.target_x = msg.data
            self.target_x_received = True
        else:
            rospy.logwarn(f"Invalid target_x received: {msg.data}")

    def lane_status_callback(self, msg):
        if msg.data in ["BOTH", "LEFT", "RIGHT", "LOST"]:
            self.lane_status = msg.data
            self.lane_status_received = True
        else:
            rospy.logwarn(f"Invalid lane_status received: {msg.data}")

    def lane_curvature_callback(self, msg):
        self.lane_curvature = msg.data

    def pose_callback(self, msg):
        if self.ego:
            self.ego.x = msg.pose.position.x
            self.ego.y = msg.pose.position.y
            q = msg.pose.orientation
            euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
            self.ego.yaw = euler[2]

    def velocity_callback(self, msg):
        if self.ego:
            self.ego.speed = msg.twist.linear.x

    def calculate_target_speed(self):
        """곡률에 따른 목표 속도 계산 - 적절한 기준값으로 수정"""
        base_speed = TARGET_SPEED
        curvature_abs = abs(self.lane_curvature)
        
        if curvature_abs > 200000:      # 매우 급한 커브
            return base_speed * 0.6
        elif curvature_abs > 100000:    # 급한 커브  
            return base_speed * 0.75
        elif curvature_abs > 50000:     # 중간 커브
            return base_speed * 0.85
        elif curvature_abs > 20000:     # 완만한 커브
            return base_speed * 0.95
        else:                           # 직선 구간
            return base_speed

    def smooth_steering(self, new_steer):
        self.steer_history.append(new_steer)
        if len(self.steer_history) > 3:
            self.steer_history.pop(0)
        smoothed = sum(self.steer_history) / len(self.steer_history)

        # 곡률 기반 최대 변화 허용 - 급커브에서 더 빠른 반응
        max_change = 8.0 + abs(self.lane_curvature / 50000.0)
        # 상한값 증가
        max_change = np.clip(max_change, 8.0, 20.0)

        if abs(smoothed - self.prev_steer) > max_change:
            smoothed = self.prev_steer + max_change * np.sign(smoothed - self.prev_steer)
        return smoothed

    def lane_following_control(self):
        ego_speed = self.ego.speed if self.ego else 5
        target_speed = self.calculate_target_speed()

        if not self.target_x_received:
            rospy.logwarn_throttle(3.0, "No target_x data - using center")
            self.target_x = 320
        if not self.lane_status_received:
            rospy.logwarn_throttle(3.0, "No lane_status data - using BOTH")
            self.lane_status = "BOTH"

        if self.lane_status == "LOST":
            rospy.logwarn_throttle(2.0, "Lane lost - maintaining straight course")
            steering_angle = self.prev_steer * 0.9
            target_speed *= 0.8
            curvature_gain = 0.0
            pid_steer = 0.0
        else:
            # PID 기본 조향각 계산
            pid_steer = self.lateral_pid.run(self.target_x, 320, 640)
            steering_angle = pid_steer

            # 곡률 기반 조향 보정 - 매우 급한 커브 대응
            if abs(self.lane_curvature) > 500000:  # 매우 급한 커브
                curvature_gain = np.clip(self.lane_curvature / 50000.0, -18.0, 18.0)
            elif abs(self.lane_curvature) > 100000:  # 급한 커브
                curvature_gain = np.clip(self.lane_curvature / 30000.0, -12.0, 12.0)
            else:  # 일반 커브
                curvature_gain = np.clip(self.lane_curvature / 15000.0, -8.0, 8.0)
            
            steering_angle += curvature_gain

            # 차선 상태별 추가 조정
            if self.lane_status == "LEFT":
                steering_angle -= 2.0
            elif self.lane_status == "RIGHT":
                steering_angle += 2.0

        final_steer = self.smooth_steering(steering_angle)
        final_steer = np.clip(final_steer, -20, 20)  # 스케일카 물리적 한계

        throttle = self.speed_pid.run(target_speed, ego_speed)

        motor_cmd = XycarMotor()
        motor_cmd.header = Header()
        motor_cmd.header.stamp = rospy.Time.now()
        motor_cmd.header.frame_id = "map"
        motor_cmd.angle = final_steer
        motor_cmd.speed = throttle

        self.prev_steer = final_steer

        pixel_error = self.target_x - 320
        rospy.loginfo_throttle(1.0, 
            f"Lane: {self.lane_status} | Error: {pixel_error:+4d}px | "
            f"PID: {pid_steer:+5.1f}° | Curve: {curvature_gain:+5.1f}° | "
            f"Final: {final_steer:+5.1f}° | Curvature: {self.lane_curvature:.0f}")
        
        return motor_cmd

    def emergency_stop(self):
        motor_cmd = XycarMotor()
        motor_cmd.header = Header()
        motor_cmd.header.stamp = rospy.Time.now()
        motor_cmd.header.frame_id = "map"
        motor_cmd.angle = 0
        motor_cmd.speed = 0
        return motor_cmd