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
TARGET_SPEED = 10  # 목표 속도 [m/s]
LANE_WIDTH = 4     # 차선 폭 [m]

# 비용 가중치
K_LATERAL = 2.0    # 횡방향 오차 가중치
K_HEADING = 1.5    # 헤딩 오차 가중치
K_VELOCITY = 1.0   # 속도 가중치
K_SMOOTHING = 0.5  # 조향 부드러움 가중치

LOOKAHEAD_OFFSET = 3

class PurePursuit:
    def __init__(self):
        self.L = 3
        self.k = 0.14
        self.Lfc = 6.0
        self.alpha = 1.5

    def euc_distance(self, pt1, pt2):
        return norm([pt2[0] - pt1[0], pt2[1] - pt1[1]])

    def global_to_local(self, target_point, position, yaw):
        dx = target_point[0] - position[0]
        dy = target_point[1] - position[1]
        x_local = dx * cos(-yaw) - dy * sin(-yaw)
        y_local = dx * sin(-yaw) + dy * cos(-yaw)
        return x_local, y_local

    def run_global(self, vEgo, target_point, position, yaw, sEgo):
        lfd = self.Lfc + self.k * vEgo
        lfd = np.clip(lfd, 5, 10)
        x_local, y_local = self.global_to_local(target_point, position, yaw)
        diff = np.sqrt(x_local**2 + y_local**2)
        if diff > 0:
            if diff >= lfd:
                theta = atan2(y_local, x_local)
                steering_angle = atan2(2 * self.L * sin(theta), lfd)
                return degrees(steering_angle), target_point
        return 0.0, target_point

class LateralPID:
    """차선 주행용 횡방향 PID 제어기"""
    def __init__(self, kp=0.8, ki=0.05, kd=0.15, dt=0.05):
        self.K_P = kp
        self.K_I = ki
        self.K_D = kd
        self.pre_error = 0.0
        self.integral_error = 0.0
        self.dt = dt

    def run(self, target_x, current_x, image_width=640):
        """
        target_x: 차선 중심 목표 X 좌표 (픽셀)
        current_x: 현재 차량 X 좌표 (이미지 중심 = image_width/2)
        """
        # 이미지 중심을 기준으로 한 횡방향 오차 계산
        lateral_error = (target_x - image_width/2) / (image_width/2)  # 정규화된 오차 [-1, 1]
        
        derivative_error = (lateral_error - self.pre_error) / self.dt
        self.integral_error += lateral_error * self.dt
        self.integral_error = np.clip(self.integral_error, -0.5, 0.5)
        
        correction = (self.K_P * lateral_error + 
                     self.K_I * self.integral_error + 
                     self.K_D * derivative_error)
        
        self.pre_error = lateral_error
        return np.clip(correction, -15, 15)  # 조향각 보정값

class SpeedPID:
    """속도 제어용 PID"""
    def __init__(self, kp=1.5, ki=0.1, kd=0.05, dt=0.05):
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
        return np.clip(throttle, 0, 15)

class LaneDrivingController:
    def __init__(self, shared, steer_ratio=12):
        self.shared = shared
        self.ego = shared.ego if hasattr(shared, 'ego') else None
        self.perception = shared.perception
        
        self.pure_pursuit = PurePursuit()
        self.lateral_pid = LateralPID(kp=0.8, ki=0.05, kd=0.15)
        self.speed_pid = SpeedPID(kp=1.5, ki=0.1, kd=0.05)
        self.steer_ratio = steer_ratio
        self.inter_steer = 0.0
        
        # 차선 정보 저장
        self.target_x = 320  # 기본값 (이미지 중심)
        self.lane_status = "LOST"
        self.lane_curvature = 0.0
        
        # ROS Subscriber 설정
        rospy.Subscriber('/lane_detection/target_x', Int32, self.target_x_callback)
        rospy.Subscriber('/lane_detection/lane_status', String, self.lane_status_callback)
        rospy.Subscriber('/lane_detection/lane_curvature', Float32, self.lane_curvature_callback)
        
        # 차량 상태 Subscriber
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_callback)
        
        # 모터 제어 발행자
        self.motor_pub = rospy.Publisher('/xycar_motor', XycarMotor, queue_size=1)
        
        rospy.loginfo("Lane Driving Controller initialized")

    def target_x_callback(self, msg):
        self.target_x = msg.data

    def lane_status_callback(self, msg):
        self.lane_status = msg.data

    def lane_curvature_callback(self, msg):
        self.lane_curvature = msg.data

    def pose_callback(self, msg):
        """차량 위치 정보 업데이트"""
        if self.ego:
            self.ego.x = msg.pose.position.x
            self.ego.y = msg.pose.position.y
            
            # Quaternion을 Euler로 변환
            orientation = msg.pose.orientation
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.ego.yaw = euler[2]

    def velocity_callback(self, msg):
        """차량 속도 정보 업데이트"""
        if self.ego:
            self.ego.speed = msg.twist.linear.x

    def calculate_target_speed(self):
        """곡률에 따른 목표 속도 계산"""
        base_speed = TARGET_SPEED
        
        # 곡률이 클수록 속도 감소
        if abs(self.lane_curvature) > 100:  # 급커브
            return base_speed * 0.6
        elif abs(self.lane_curvature) > 50:  # 중간 커브
            return base_speed * 0.8
        else:  # 직선 또는 완만한 커브
            return base_speed

    def lane_following_control(self):
        """
        차선 추종 제어 - Shared 객체 사용
        """
        # Ego 정보 가져오기
        if self.ego:
            ego_x = self.ego.x
            ego_y = self.ego.y
            ego_yaw = self.ego.yaw
            ego_speed = self.ego.speed
        else:
            # Ego 객체가 없으면 기본값 사용
            ego_x, ego_y, ego_yaw, ego_speed = 0, 0, 0, 5
        
        # 목표 속도 계산
        target_speed = self.calculate_target_speed()
        
        # 차선 상태에 따른 제어 로직
        if self.lane_status == "LOST":
            rospy.logwarn("Lane lost! Using safe default control")
            steering_correction = 0.0
            target_speed *= 0.7  # 속도 감소
        else:
            # 횡방향 PID 제어로 조향각 보정
            steering_correction = self.lateral_pid.run(self.target_x, 320, 640)
            
            # 차선 상태에 따른 추가 보정
            if self.lane_status == "LEFT":
                # 왼쪽 차선만 보일 때 약간 우측으로 보정
                steering_correction -= 2.0
            elif self.lane_status == "RIGHT":
                # 오른쪽 차선만 보일 때 약간 좌측으로 보정
                steering_correction += 2.0

        # 속도 제어
        throttle = self.speed_pid.run(target_speed, ego_speed)
        
        # 최종 조향각 계산
        final_steer = steering_correction * self.steer_ratio
        
        # 조향각 부드러움을 위한 선형 보간
        steer_interp = np.linspace(self.inter_steer, final_steer, 5)  # 더 빠른 응답을 위해 5로 축소
        
        # ROS 메시지 생성 및 발행
        for i, s in enumerate(steer_interp):
            motor_cmd = XycarMotor()
            motor_cmd.header = Header()
            motor_cmd.header.stamp = rospy.Time.now()
            motor_cmd.header.frame_id = "map"
            motor_cmd.angle = s
            motor_cmd.speed = throttle
            
            # 마지막 명령만 발행 (또는 모든 명령을 짧은 간격으로 발행)
            if i == len(steer_interp) - 1:
                self.motor_pub.publish(motor_cmd)

        self.inter_steer = final_steer
        
        # 디버그 정보 출력
        rospy.loginfo_throttle(1.0, 
            f"Lane Status: {self.lane_status}, Target X: {self.target_x}, "
            f"Steer Correction: {steering_correction:.2f}, Throttle: {throttle:.2f}, "
            f"Curvature: {self.lane_curvature:.2f}")
        
        return motor_cmd

    def emergency_stop(self):
        """비상 정지"""
        motor_cmd = XycarMotor()
        motor_cmd.header = Header()
        motor_cmd.header.stamp = rospy.Time.now()
        motor_cmd.header.frame_id = "map"
        motor_cmd.angle = 0
        motor_cmd.speed = 0
        self.motor_pub.publish(motor_cmd)
        return motor_cmd

    def run_lane_driving(self):
        """메인 차선 주행 루프"""
        rate = rospy.Rate(20)  # 20Hz
        
        while not rospy.is_shutdown():
            try:
                # 차선 추종 제어 실행
                control_command = self.lane_following_control()
                
                # Shared 객체에 현재 제어 상태 저장 (필요시)
                if hasattr(self.shared, 'plan'):
                    self.shared.plan.motion_decision = "lane_following"
                    
            except Exception as e:
                rospy.logerr(f"Lane driving control error: {e}")
                # 비상 정지
                self.emergency_stop()
            
            rate.sleep()

def standalone_lane_driving():
    """독립적으로 차선 주행 제어를 실행하는 함수"""
    rospy.init_node('lane_driving_controller')
    
    # 간단한 Shared 객체 생성
    class SimpleShared:
        def __init__(self):
            from ego import Ego
            from perception import Perception
            self.ego = Ego()
            self.perception = Perception()
    
    shared = SimpleShared()
    controller = LaneDrivingController(shared, steer_ratio=12)
    
    try:
        controller.run_lane_driving()
    except rospy.ROSInterruptException:
        rospy.loginfo("Lane driving controller stopped")

class LaneDrivingPlanner:
    def __init__(self, shared, rate=20):
        self.shared = shared
        self.period = 1.0 / rate
        self.controller = LaneDrivingController(shared)
        
    def run(self):
        """Threading으로 실행될 메인 루프"""
        while True:
            try:
                if hasattr(self.shared, 'plan') and self.shared.plan.motion_decision == "lane_following":
                    self.controller.lane_following_control()
                    
            except Exception as e:
                rospy.logerr(f"Lane driving planner error: {e}")
                self.controller.emergency_stop()
                
            rospy.sleep(self.period)

# 사용 예시
if __name__ == '__main__':
    # 독립 실행
    standalone_lane_driving()