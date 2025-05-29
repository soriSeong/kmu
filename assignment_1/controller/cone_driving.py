#!/usr/bin/env python3
import rospy
import math
import numpy as np
from xycar_msgs.msg import XycarMotor
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class ConeDrivingController:
    def __init__(self, shared):
        self.shared = shared
        self.ego = shared.ego
        self.perception = shared.perception
        
        # 제어 파라미터
        self.LOOKAHEAD_DISTANCE = 2.0  # 전방 주시 거리
        self.MAX_STEER_ANGLE = 30.0    # 최대 조향각 (도)
        self.SPEED_NORMAL = 20         # 정상 속도
        self.SPEED_CURVE = 15          # 곡선 구간 속도
        self.SPEED_SHARP_CURVE = 10    # 급곡선 속도
        
        # Pure Pursuit 파라미터
        self.wheelbase = 1.04  # 휠베이스
            
        # 경로 시각화를 위한 퍼블리셔
        self.path_pub = rospy.Publisher('/cone_path', Path, queue_size=1)
        
        rospy.loginfo("Cone Driving Controller initialized")
    
    def follow_cone_path(self):
        """라바콘 경로 추종 메인 함수"""
        try:
            if not self.perception.middle_path:
                rospy.logwarn("No middle path available")
                return None
                
            # 현재 차량 위치
            current_x = self.ego.x if self.ego else 0
            current_y = self.ego.y if self.ego else 0
            current_yaw = self.ego.yaw if self.ego else 0
            current_speed = self.ego.speed if self.ego else 0
            
            # 목표점 찾기 (Pure Pursuit)
            target_point = self.find_target_point(current_x, current_y)
            
            if target_point is None:
                rospy.logwarn("No target point found")
                return self.create_motor_command(0, 10)  # 직진, 감속
            
            # 조향각 계산
            steer_angle = self.calculate_steer_angle(
                current_x, current_y, current_yaw, target_point
            )
            
            # 속도 계산 (곡률에 따라 조정)
            target_speed = self.calculate_target_speed(steer_angle, current_speed)
            
            # 경로 시각화
            self.publish_path_visualization()
            
            # 모터 명령 생성
            return self.create_motor_command(steer_angle, target_speed)
            
        except Exception as e:
            rospy.logerr(f"Cone path following error: {e}")
            return self.create_motor_command(0, 10)  # 비상 명령
    
    def find_target_point(self, current_x, current_y):
        """Pure Pursuit 알고리즘으로 목표점 선택"""
        if not self.perception.middle_path:
            return None
            
        # 전방 주시 거리 내의 점들 중에서 가장 먼 점 선택
        target_point = None
        max_distance = 0
        
        for point in self.perception.middle_path:
            px, py = point
            distance = math.sqrt((px - current_x)**2 + (py - current_y)**2)
            
            # 전방 주시 거리 범위 내에서 가장 먼 점 선택
            if distance >= self.LOOKAHEAD_DISTANCE * 0.8 and distance <= self.LOOKAHEAD_DISTANCE * 1.5:
                if distance > max_distance:
                    max_distance = distance
                    target_point = (px, py)
        
        # 적절한 점이 없으면 가장 가까운 점 선택
        if target_point is None and self.perception.middle_path:
            min_distance = float('inf')
            for point in self.perception.middle_path:
                px, py = point
                distance = math.sqrt((px - current_x)**2 + (py - current_y)**2)
                if distance < min_distance:
                    min_distance = distance
                    target_point = (px, py)
        
        return target_point
    
    def calculate_steer_angle(self, current_x, current_y, current_yaw, target_point):
        """Pure Pursuit 조향각 계산"""
        if target_point is None:
            return 0.0
            
        tx, ty = target_point
        
        # 차량 좌표계로 변환
        dx = tx - current_x
        dy = ty - current_y
        
        # 차량 기준 좌표계로 회전
        local_x = dx * math.cos(current_yaw) + dy * math.sin(current_yaw)
        local_y = -dx * math.sin(current_yaw) + dy * math.cos(current_yaw)
        
        # 전방 거리 계산
        distance = math.sqrt(local_x**2 + local_y**2)
        
        if distance < 0.1:  # 너무 가까우면 직진
            return 0.0
        
        # Pure Pursuit 조향각 계산
        alpha = math.atan2(local_y, local_x)
        steer_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), distance)
        
        # 조향각을 도 단위로 변환하고 제한
        steer_angle_deg = math.degrees(steer_angle)
        steer_angle_deg = max(-self.MAX_STEER_ANGLE, 
                            min(self.MAX_STEER_ANGLE, steer_angle_deg))
        
        return steer_angle_deg
    
    def calculate_target_speed(self, steer_angle, current_speed):
        """조향각에 따른 목표 속도 계산"""
        abs_steer = abs(steer_angle)
        
        if abs_steer > 20:  # 급곡선
            target_speed = self.SPEED_SHARP_CURVE
        elif abs_steer > 10:  # 곡선
            target_speed = self.SPEED_CURVE
        else:  # 직선
            target_speed = self.SPEED_NORMAL
        
        # 급격한 속도 변화 방지 (부드러운 가속/감속)
        speed_diff = target_speed - current_speed
        if abs(speed_diff) > 5:
            if speed_diff > 0:
                target_speed = current_speed + 3  # 점진적 가속
            else:
                target_speed = current_speed - 3  # 점진적 감속
        
        return max(5, min(25, target_speed))  # 속도 제한
    
    def create_motor_command(self, steer_angle, speed):
        """모터 제어 명령 생성"""
        motor_msg = XycarMotor()
        motor_msg.header = Header()
        motor_msg.header.stamp = rospy.Time.now()
        motor_msg.header.frame_id = "map"
        motor_msg.angle = int(steer_angle)
        motor_msg.speed = int(speed)
        
        return motor_msg
    
    def publish_path_visualization(self):
        """경로 시각화를 위한 퍼블리시"""
        if not self.perception.middle_path:
            return
            
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for point in self.perception.middle_path:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def get_path_curvature(self, path_points, index):
        """경로의 곡률 계산 (3점을 이용)"""
        if len(path_points) < 3 or index < 1 or index >= len(path_points) - 1:
            return 0.0
            
        p1 = path_points[index - 1]
        p2 = path_points[index]
        p3 = path_points[index + 1]
        
        # 3점을 이용한 곡률 계산
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        
        try:
            # 곡률 계산 공식
            a = math.sqrt((x2-x1)**2 + (y2-y1)**2)
            b = math.sqrt((x3-x2)**2 + (y3-y2)**2)
            c = math.sqrt((x1-x3)**2 + (y1-y3)**2)
            
            if a == 0 or b == 0:
                return 0.0
                
            # 삼각형의 넓이를 이용한 곡률 계산
            s = (a + b + c) / 2
            area = math.sqrt(s * (s-a) * (s-b) * (s-c))
            curvature = 4 * area / (a * b * c)
            
            return curvature
            
        except:
            return 0.0
    
    def emergency_stop(self):
        """비상 정지"""
        rospy.logwarn("Cone driving emergency stop!")
        return self.create_motor_command(0, 0)
    
    def get_status(self):
        """현재 상태 반환"""
        return {
            'middle_path_length': len(self.perception.middle_path) if self.perception.middle_path else 0,
            'lookahead_distance': self.LOOKAHEAD_DISTANCE,
            'max_steer_angle': self.MAX_STEER_ANGLE
        }