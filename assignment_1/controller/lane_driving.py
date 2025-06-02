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
TARGET_SPEED = 35  # 목표 속도
IMAGE_WIDTH = 640  # 이미지 너비
IMAGE_CENTER = 320  # 이미지 중심

class LateralPIDRelative:
    """상대좌표 기준 횡방향 PID 제어기"""
    def __init__(self, kp=1.5, ki=0.05, kd=0.2, dt=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.previous_error = 0.0
        self.integral = 0.0
        self.error_history = []  # 에러 히스토리

    def update_from_pixel_error(self, target_x, current_x=IMAGE_CENTER, image_width=IMAGE_WIDTH):
        """픽셀 오차를 상대좌표 에러로 변환하여 제어"""
        # 픽셀 오차 계산
        pixel_error = target_x - current_x
        
        # 에러 히스토리 관리 (이동평균)
        self.error_history.append(pixel_error)
        if len(self.error_history) > 5:
            self.error_history.pop(0)
        
        # 이동평균으로 노이즈 감소
        smoothed_error = sum(self.error_history) / len(self.error_history)
        
        # 픽셀 오차를 상대 거리 에러로 변환 (정규화)
        # 이미지 중심에서 픽셀당 약 0.01m로 근사
        lateral_distance_error = (smoothed_error / (image_width/4)) * 2.0  # 상대거리 [m]
        
        # 데드존 설정 (작은 오차 무시)
        if abs(lateral_distance_error) < 0.2:  # 20cm 이내는 미세조정
            lateral_distance_error *= 0.5
        
        return self.update_lateral_error(lateral_distance_error)

    def update_lateral_error(self, lateral_error):
        """상대좌표 기준 횡방향 에러로 PID 제어"""
        derivative = (lateral_error - self.previous_error) / self.dt
        self.integral += lateral_error * self.dt
        self.integral = np.clip(self.integral, -5.0, 5.0)  # 적분 제한
        
        # PID 출력 계산
        output = (self.kp * lateral_error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        self.previous_error = lateral_error
        return np.clip(output, -20, 20)  # 조향각 제한

class SpeedPIDRelative:
    """상대좌표 기준 속도 제어용 PID"""
    def __init__(self, kp=1.5, ki=0.05, kd=0.2, dt=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.previous_error = 0.0
        self.integral = 0.0

    def update_speed_error(self, target_speed, current_speed):
        """속도 에러 기반 PID 제어"""
        speed_error = target_speed - current_speed
        
        derivative = (speed_error - self.previous_error) / self.dt
        self.integral += speed_error * self.dt
        self.integral = np.clip(self.integral, -5, 5)
        
        output = (self.kp * speed_error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        self.previous_error = speed_error
        return max(0, np.clip(output, 0, 40))  # 양수 속도만 출력

class LaneDrivingController:
    """상대좌표 기준 차선 주행 제어기"""
    def __init__(self, shared, steer_ratio=1.0):
        self.shared = shared
        self.ego = shared.ego if hasattr(shared, 'ego') else None
        self.perception = shared.perception
        
        # 상대좌표 기준 PID 제어기
        self.lateral_pid = LateralPIDRelative(kp=1.9, ki=0.03, kd=0.4)
        self.speed_pid = SpeedPIDRelative(kp=1.5, ki=0.05, kd=0.2)
        self.steer_ratio = steer_ratio
        
        # 조향각 스무딩
        self.prev_steer = 0.0
        self.steer_history = []
        
        # 차선 인식 상태 (상대좌표 기준)
        self.target_x = IMAGE_CENTER  # 픽셀 좌표
        self.lane_status = "BOTH"
        self.lane_curvature = 0.0  # 곡률
        
        # 커브 방향 판단을 위한 간단한 방법 (target_x 위치 기준)
        self.curve_direction = 0  # -1: 좌회전, 0: 직진, 1: 우회전
        
        # 데이터 수신 플래그
        self.target_x_received = False
        self.lane_status_received = False
        
        # ROS 구독자들
        rospy.Subscriber('/lane_detection/target_x', Int32, self.target_x_callback)
        rospy.Subscriber('/lane_detection/lane_status', String, self.lane_status_callback)
        rospy.Subscriber('/lane_detection/lane_curvature', Float32, self.lane_curvature_callback)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_callback)
        
        rospy.loginfo("Lane Driving Controller initialized (Curve Direction Detection)")

    def target_x_callback(self, msg):
        """차선 중심 픽셀 좌표 수신"""
        if 50 <= msg.data <= 590:  # 유효 범위 체크
            self.target_x = msg.data
            self.target_x_received = True
                
        else:
            rospy.logwarn(f"Invalid target_x received: {msg.data}")

    def lane_status_callback(self, msg):
        """차선 상태 수신"""
        if msg.data in ["BOTH", "LEFT", "RIGHT", "LOST"]:
            self.lane_status = msg.data
            self.lane_status_received = True
        else:
            rospy.logwarn(f"Invalid lane_status received: {msg.data}")

    def lane_curvature_callback(self, msg):
        """차선 곡률 수신"""
        self.lane_curvature = msg.data

    def pose_callback(self, msg):
        """차량 위치 업데이트 (절대좌표)"""
        if self.ego:
            self.ego.x = msg.pose.position.x
            self.ego.y = msg.pose.position.y
            q = msg.pose.orientation
            euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
            self.ego.yaw = euler[2]

    def velocity_callback(self, msg):
        """차량 속도 업데이트"""
        if self.ego:
            self.ego.speed = msg.twist.linear.x

    def detect_curve_direction(self):
        """target_x 위치로 커브 방향 판단 (간단한 방법)"""
        # 이미지 중심(320)을 기준으로 방향 판단
        if self.target_x < IMAGE_CENTER:  # 320보다 작으면 좌회전
            return -1  # 좌회전
        elif self.target_x > IMAGE_CENTER:  # 320보다 크면 우회전
            return 1   # 우회전
        else:  # 정확히 중심이면 직진
            return 0   # 직진

    def calculate_target_speed_relative(self):
        """상대좌표 기준 곡률에 따른 목표 속도 계산"""
        base_speed = TARGET_SPEED
        curvature_abs = abs(self.lane_curvature)
        # 곡률 기준 속도 조정 (상대좌표 기준 임계값)
        if curvature_abs > 1000000:  # 곡선이 튀거나 급한 커브
            return base_speed * 0.3
        if curvature_abs > 700000:      # 매우 급한 커브
            return base_speed * 0.4
        elif curvature_abs > 500000:    # 급한 커브  
            return base_speed * 0.5
        elif curvature_abs < 300000:    
            return base_speed * 0.6
        elif curvature_abs < 200000:    # 일반 커브
            return base_speed * 0.7
        else:                           # 직선 구간
            return base_speed   

    def smooth_steering_relative(self, new_steer):
        """상대좌표 기준 조향각 부드럽게 급변하는 것을 방지"""
        self.steer_history.append(new_steer)
        if len(self.steer_history) > 3:
            self.steer_history.pop(0)
        
        # 이동평균
        smoothed = sum(self.steer_history) / len(self.steer_history)

        # 곡률 기반 최대 변화량 조정
        max_change = 8.0 + abs(self.lane_curvature / 50000.0)
        max_change = np.clip(max_change, 8.0, 20.0)

        # 급격한 변화 제한
        if abs(smoothed - self.prev_steer) > max_change:
            smoothed = self.prev_steer + max_change * np.sign(smoothed - self.prev_steer)
        
        return smoothed

    def lane_following_control(self):
        """상대좌표 기준 차선 추종 제어"""
        # 차량 상태 가져오기
        ego_speed = self.ego.speed if self.ego else 5
        target_speed = self.calculate_target_speed_relative()

        # 데이터 유효성 체크
        if not self.target_x_received:
            self.target_x = IMAGE_CENTER
        if not self.lane_status_received:
            self.lane_status = "BOTH"

        # 커브 방향 감지
        self.curve_direction = self.detect_curve_direction()

        # 차선 상태에 따른 제어
        if self.lane_status == "LOST":
            steering_angle = self.prev_steer * 0.9  # 점진적으로 직진
            target_speed *= 0.8  # 감속
            curvature_gain = 0.0
            pid_steer = 0.0
        else:
            # 상대좌표 기준 PID 조향각 계산
            pid_steer = self.lateral_pid.update_from_pixel_error(
                self.target_x, IMAGE_CENTER, IMAGE_WIDTH
            )
            steering_angle = pid_steer

            # 곡률 기반 조향 보정 (방향 고려)
            curvature_gain = 0.0
            if self.lane_curvature > 700000:  # 매우 급한 커브
                base_gain = self.lane_curvature / 700000.0
                curvature_gain = base_gain * self.curve_direction  # 방향 적용
                curvature_gain = np.clip(curvature_gain, -20.0, 20.0)
            elif self.lane_curvature > 500000:  # 급한 커브
                base_gain = self.lane_curvature / 500000.0
                curvature_gain = base_gain * self.curve_direction  # 방향 적용
                curvature_gain = np.clip(curvature_gain, -17.0, 17.0)
            elif self.lane_curvature > 300000:  # 급한 커브2
                base_gain = self.lane_curvature / 300000.0
                curvature_gain = base_gain * self.curve_direction  # 방향 적용
                curvature_gain = np.clip(curvature_gain, -14.0, 14.0)
            elif self.lane_curvature > 200000:  # 급한 커브
                base_gain = self.lane_curvature / 200000.0
                curvature_gain = base_gain * self.curve_direction  # 방향 적용
                curvature_gain = np.clip(curvature_gain, -12.0, 12.0)
            elif self.lane_curvature > 150000:  # 일반 커브
                base_gain = self.lane_curvature / 150000.0
                curvature_gain = base_gain * self.curve_direction  # 방향 적용
                curvature_gain = np.clip(curvature_gain, -10.0, 10.0)
            
            steering_angle += curvature_gain

            # 차선 상태별 추가 조정 (상대좌표 기준)
            if self.lane_status == "LEFT":  # 왼쪽 차선만 보임 -> 우측으로 이동
                steering_angle += 5.0
            elif self.lane_status == "RIGHT":  # 오른쪽 차선만 보임 -> 좌측으로 이동
                steering_angle -= 5.0

        # 조향각 스무딩
        final_steer = self.smooth_steering_relative(steering_angle)
        final_steer = np.clip(final_steer, -20, 20)

        # 속도 제어
        throttle = self.speed_pid.update_speed_error(target_speed, ego_speed)

        # 모터 명령 생성
        motor_cmd = XycarMotor()
        motor_cmd.header = Header()
        motor_cmd.header.stamp = rospy.Time.now()
        motor_cmd.header.frame_id = "map"
        motor_cmd.angle = int(final_steer * 5)
        motor_cmd.speed = int(throttle)

        self.prev_steer = final_steer

        # 상대좌표 기준 디버그 로그 (방향 정보 추가)
        pixel_error = self.target_x - IMAGE_CENTER
        lateral_distance_error = (pixel_error / (IMAGE_WIDTH/4)) * 2.0
        curve_direction_str = "LEFT" if self.curve_direction == -1 else "RIGHT" if self.curve_direction == 1 else "STRAIGHT"
        
        rospy.loginfo_throttle(1.0, 
            f"Lane - Status: {self.lane_status} | Direction: {curve_direction_str} | "
            f"Pixel Error: {pixel_error:+4d}px | "
            f"PID: {pid_steer:+5.1f}° | Curve: {curvature_gain:+5.1f}° | "
            f"Final: {final_steer:+5.1f}° | Curvature: {self.lane_curvature:.0f}")
        
        return motor_cmd

    def emergency_stop(self):
        """비상 정지"""
        rospy.logwarn("Lane driving emergency stop!")
        motor_cmd = XycarMotor()
        motor_cmd.header = Header()
        motor_cmd.header.stamp = rospy.Time.now()
        motor_cmd.header.frame_id = "map"
        motor_cmd.angle = 0
        motor_cmd.speed = 0
        return motor_cmd

    def get_status_relative(self):
        """상대좌표 기준 상태 반환"""
        pixel_error = self.target_x - IMAGE_CENTER
        lateral_distance_error = (pixel_error / (IMAGE_WIDTH/4)) * 2.0
        curve_direction_str = "LEFT" if self.curve_direction == -1 else "RIGHT" if self.curve_direction == 1 else "STRAIGHT"
        
        return {
            'coordinate_system': 'relative',
            'pixel_error': pixel_error,
            'lateral_distance_error_m': lateral_distance_error,
            'lane_status': self.lane_status,
            'lane_curvature': self.lane_curvature,
            'curve_direction': curve_direction_str,
            'target_x_px': self.target_x,
            'image_center_px': IMAGE_CENTER
        }

    def reset_controller(self):
        """제어기 상태 초기화"""
        self.lateral_pid.previous_error = 0.0
        self.lateral_pid.integral = 0.0
        self.lateral_pid.error_history = []
        
        self.speed_pid.previous_error = 0.0
        self.speed_pid.integral = 0.0
        
        self.prev_steer = 0.0
        self.steer_history = []