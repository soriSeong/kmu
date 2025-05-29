#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from math import atan2, sin, cos, degrees, sqrt
from std_msgs.msg import Header
from xycar_msgs.msg import XycarMotor
from numpy.linalg import norm
import rospy

# 파라미터 설정
TARGET_SPEED = 10  # 목표 속도 [m/s]
LOOKAHEAD_OFFSET = 3  # 전방 주시 오프셋

class PurePursuitRelative:
    """상대좌표 기준 Pure Pursuit"""
    def __init__(self):
        self.wheelbase = 1.04  # 휠베이스 [m]
        self.lookahead_gain = 0.14
        self.min_lookahead = 3.0  # 최소 전방 주시 거리
        self.max_lookahead = 10.0  # 최대 전방 주시 거리

    def calculate_lookahead_distance(self, velocity):
        """속도에 따른 전방 주시 거리 계산"""
        lookahead = self.min_lookahead + self.lookahead_gain * velocity
        return np.clip(lookahead, self.min_lookahead, self.max_lookahead)

    def global_to_relative(self, global_path, ego_x, ego_y, ego_yaw):
        """절대좌표 경로를 상대좌표로 변환"""
        relative_path = []
        
        for gx, gy in zip(global_path.x, global_path.y):
            # 차량 기준으로 평행이동
            dx = gx - ego_x
            dy = gy - ego_y
            
            # 차량 heading 기준으로 회전 변환
            rel_x = dx * cos(-ego_yaw) - dy * sin(-ego_yaw)
            rel_y = dx * sin(-ego_yaw) + dy * cos(-ego_yaw)
            
            relative_path.append((rel_x, rel_y))
        
        return relative_path

    def find_target_point_relative(self, relative_path, velocity):
        """상대좌표 기준 목표점 찾기"""
        if not relative_path:
            return None
        
        # 전방(x > 0) 점들만 고려
        forward_points = [(x, y, i) for i, (x, y) in enumerate(relative_path) if x > 0.1]
        
        if not forward_points:
            return None
        
        lookahead_distance = self.calculate_lookahead_distance(velocity)
        
        # 전방 주시 거리에 가장 가까운 점 찾기
        best_point = None
        min_diff = float('inf')
        
        for x, y, idx in forward_points:
            distance = sqrt(x**2 + y**2)
            diff = abs(distance - lookahead_distance)
            
            if diff < min_diff:
                min_diff = diff
                best_point = (x, y, idx)
        
        return best_point

    def calculate_steering_angle_relative(self, target_point):
        """상대좌표 기준 Pure Pursuit 조향각 계산"""
        if target_point is None:
            return 0.0
        
        rel_x, rel_y, _ = target_point
        
        # 거리 계산
        distance = sqrt(rel_x**2 + rel_y**2)
        
        if distance < 0.1:
            return 0.0
        
        # Pure Pursuit 조향각 계산
        alpha = atan2(rel_y, rel_x)
        steering_angle = atan2(2.0 * self.wheelbase * sin(alpha), distance)
        
        return degrees(steering_angle)

class PIDRelative:
    """상대좌표 기준 PID 제어기"""
    def __init__(self, kp, ki, kd, dt=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.previous_error = 0.0
        self.integral = 0.0

    def update_relative(self, target_point, current_speed):
        """상대좌표 기준 PID 업데이트"""
        if target_point is None:
            return 0.0
        
        rel_x, rel_y, _ = target_point
        
        # 목표점까지의 거리를 에러로 사용
        distance_error = sqrt(rel_x**2 + rel_y**2)
        
        # 속도 제어용 에러 (거리가 멀수록 더 빠르게)
        speed_error = max(0, distance_error - 2.0)  # 2m 이내는 감속
        
        derivative = (speed_error - self.previous_error) / self.dt
        self.integral += speed_error * self.dt
        self.integral = np.clip(self.integral, -5, 5)
        
        output = (self.kp * speed_error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        self.previous_error = speed_error
        return np.clip(output, 0, TARGET_SPEED)

class FrenetController:
    """상대좌표 기준 Frenet 제어기"""
    def __init__(self, steer_ratio=12):
        self.pure_pursuit = PurePursuitRelative()
        self.pid = PIDRelative(kp=1.0, ki=0.1, kd=0.01)
        self.steer_ratio = steer_ratio
        self.inter_steer = 0.0

    def follow_frenet_path(self, path, ego_x, ego_y, yaw, speed, steer_prev):
        """상대좌표 기준 Frenet 경로 추종"""
        try:
            # 절대좌표 경로를 상대좌표로 변환
            relative_path = self.pure_pursuit.global_to_relative(
                path, ego_x, ego_y, yaw
            )
            
            if not relative_path:
                rospy.logwarn("No relative path available")
                return None
            
            # 상대좌표 기준 목표점 찾기
            target_point = self.pure_pursuit.find_target_point_relative(
                relative_path, speed
            )
            
            if target_point is None:
                rospy.logwarn("No target point found in relative coordinate")
                return None
            
            # 상대좌표 기준 조향각 계산
            steer_deg = self.pure_pursuit.calculate_steering_angle_relative(target_point)
            steer = steer_deg / self.steer_ratio  # 스케일 조정
            
            # 상대좌표 기준 속도 계산
            throttle = self.pid.update_relative(target_point, speed)
            throttle = np.clip(throttle, 0, 15)  # Frenet 주행은 보수적으로
            
            # 조향각 보간으로 부드러운 제어
            steer_interp = np.linspace(self.inter_steer, steer, 10)
            motor_commands = []
            
            for s in steer_interp:
                motor_msg = XycarMotor()
                motor_msg.header = Header()
                motor_msg.header.stamp = rospy.Time.now()
                motor_msg.header.frame_id = "map"
                motor_msg.angle = int(s)
                motor_msg.speed = int(throttle)
                motor_commands.append(motor_msg)
            
            self.inter_steer = steer
            
            # 디버그 로그
            rel_x, rel_y, path_idx = target_point
            rospy.loginfo_throttle(1.0, 
                f"Frenet Relative - Target: ({rel_x:.2f}, {rel_y:.2f}) | "
                f"Steer: {steer_deg:.1f}° | Speed: {throttle:.1f}")
            
            return motor_commands
            
        except Exception as e:
            rospy.logerr(f"Frenet relative control error: {e}")
            return None

    def get_closest_waypoint_relative(self, relative_path):
        """상대좌표 기준 가장 가까운 웨이포인트 찾기"""
        if not relative_path:
            return 0
        
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (rel_x, rel_y) in enumerate(relative_path):
            # 차량 위치는 (0, 0)이므로 단순히 거리 계산
            dist = sqrt(rel_x**2 + rel_y**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        return closest_idx

    def reset(self):
        """제어기 상태 초기화"""
        self.inter_steer = 0.0
        self.pid.previous_error = 0.0
        self.pid.integral = 0.0