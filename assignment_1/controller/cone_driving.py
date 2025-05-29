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
        
        # 제어 파라미터 (상대좌표 기준)
        self.LOOKAHEAD_DISTANCE = 2.0  # 전방 주시 거리 [m]
        self.MAX_STEER_ANGLE = 30.0    # 최대 조향각 [도]
        self.SPEED_NORMAL = 20         # 정상 속도
        self.SPEED_CURVE = 15          # 곡선 구간 속도
        self.SPEED_SHARP_CURVE = 10    # 급곡선 속도
        
        # Pure Pursuit 파라미터
        self.wheelbase = 1.04  # 휠베이스 [m]
            
        # 경로 시각화를 위한 퍼블리셔
        self.path_pub = rospy.Publisher('/cone_path', Path, queue_size=1)
        
        rospy.loginfo("Cone Driving Controller initialized (Relative Coordinate)")
    
    def global_to_relative(self, global_points, ego_x, ego_y, ego_yaw):
        """절대좌표를 차량 기준 상대좌표로 변환"""
        relative_points = []
        
        for gx, gy in global_points:
            # 차량 기준으로 평행이동
            dx = gx - ego_x
            dy = gy - ego_y
            
            # 차량 heading 기준으로 회전 변환
            rel_x = dx * math.cos(-ego_yaw) - dy * math.sin(-ego_yaw)
            rel_y = dx * math.sin(-ego_yaw) + dy * math.cos(-ego_yaw)
            
            relative_points.append((rel_x, rel_y))
        
        return relative_points
    
    def follow_cone_path(self):
        """라바콘 경로 추종 메인 함수 (상대좌표 기준)"""
        try:
            if not self.perception.middle_path:
                rospy.logwarn("No middle path available")
                return None
                
            # 현재 차량 상태 (절대좌표)
            ego_x = self.ego.x if self.ego else 0
            ego_y = self.ego.y if self.ego else 0
            ego_yaw = self.ego.yaw if self.ego else 0
            ego_speed = self.ego.speed if self.ego else 0
            
            # 절대좌표 경로를 상대좌표로 변환
            relative_path = self.global_to_relative(
                self.perception.middle_path, ego_x, ego_y, ego_yaw
            )
            
            # 상대좌표 기준으로 목표점 찾기
            target_point = self.find_target_point_relative(relative_path)
            
            if target_point is None:
                rospy.logwarn("No target point found")
                return self.create_motor_command(0, 10)  # 직진, 감속
            
            # 상대좌표 기준 조향각 계산
            steer_angle = self.calculate_steer_angle_relative(target_point)
            
            # 속도 계산 (곡률에 따라 조정)
            target_speed = self.calculate_target_speed(steer_angle, ego_speed)
            
            # 경로 시각화 (절대좌표로 다시 변환해서 퍼블리시)
            self.publish_path_visualization()
            
            # 모터 명령 생성
            return self.create_motor_command(steer_angle, target_speed)
            
        except Exception as e:
            rospy.logerr(f"Cone path following error: {e}")
            return self.create_motor_command(0, 10)  # 비상 명령
    
    def find_target_point_relative(self, relative_path):
        """상대좌표 기준 Pure Pursuit 목표점 선택"""
        if not relative_path:
            return None
            
        # 전방(x > 0) 영역에서만 목표점 탐색
        forward_points = [(x, y) for x, y in relative_path if x > 0.1]
        
        if not forward_points:
            return None
        
        # 전방 주시 거리 범위 내에서 가장 먼 점 선택
        target_point = None
        max_distance = 0
        
        for rel_x, rel_y in forward_points:
            distance = math.sqrt(rel_x**2 + rel_y**2)
            
            # 전방 주시 거리 범위 확인
            if (self.LOOKAHEAD_DISTANCE * 0.8 <= distance <= 
                self.LOOKAHEAD_DISTANCE * 1.5):
                if distance > max_distance:
                    max_distance = distance
                    target_point = (rel_x, rel_y)
        
        # 적절한 점이 없으면 가장 가까운 전방 점 선택
        if target_point is None and forward_points:
            min_distance = float('inf')
            for rel_x, rel_y in forward_points:
                distance = math.sqrt(rel_x**2 + rel_y**2)
                if distance < min_distance:
                    min_distance = distance
                    target_point = (rel_x, rel_y)
        
        return target_point
    
    def calculate_steer_angle_relative(self, target_point):
        """상대좌표 기준 Pure Pursuit 조향각 계산"""
        if target_point is None:
            return 0.0
            
        rel_x, rel_y = target_point
        
        # 전방 거리 계산
        distance = math.sqrt(rel_x**2 + rel_y**2)
        
        if distance < 0.1:  # 너무 가까우면 직진
            return 0.0
        
        # Pure Pursuit 조향각 계산 (이미 상대좌표이므로 바로 계산)
        alpha = math.atan2(rel_y, rel_x)
        steer_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), distance)
        
        # 조향각을 도 단위로 변환하고 제한
        steer_angle_deg = math.degrees(steer_angle)
        steer_angle_deg = max(-self.MAX_STEER_ANGLE, 
                            min(self.MAX_STEER_ANGLE, steer_angle_deg))
        
        rospy.loginfo_throttle(1.0, 
            f"Cone Target: ({rel_x:.2f}, {rel_y:.2f}) | "
            f"Distance: {distance:.2f}m | Steer: {steer_angle_deg:.1f}°")
        
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
        """경로 시각화를 위한 퍼블리시 (절대좌표로 변환)"""
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
    
    def get_path_curvature_relative(self, relative_path, target_index):
        """상대좌표 기준 경로 곡률 계산"""
        if (len(relative_path) < 3 or target_index < 1 or 
            target_index >= len(relative_path) - 1):
            return 0.0
            
        p1 = relative_path[target_index - 1]
        p2 = relative_path[target_index]
        p3 = relative_path[target_index + 1]
        
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
            if s * (s-a) * (s-b) * (s-c) <= 0:
                return 0.0
                
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
            'max_steer_angle': self.MAX_STEER_ANGLE,
            'coordinate_system': 'relative'
        }