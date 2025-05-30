#!/usr/bin/env python3
import rospy

import numpy as np
from xycar_msgs.msg import XycarMotor
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from math import atan2, degrees

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
            
            
            # 목표점 찾기 (Pure Pursuit)
            target_point = self.perception.middle_path[0]
            rospy.loginfo(f"[Cone Driving] Target point (relative): x={target_point[0]:.2f}, y={target_point[1]:.2f}")
            
            # 조향각 계산
            steer_angle = self.pure_pursuit_from_relative_point(target_point)
            
            # 속도 계산 (곡률에 따라 조정)
            target_speed = 7
            
            
            
            # 모터 명령 생성
            return self.create_motor_command(steer_angle*3.6, target_speed)

        except Exception as e:
            rospy.logerr(f"Cone path following error: {e}")
            return self.create_motor_command(0, 10)  # 비상 명령
    

    def pure_pursuit_from_relative_point(self,target_point):
        """
        상대좌표 기준 Pure Pursuit 함수
        target_point: (x, y) 형태의 튜플 (상대 좌표 기준)
        반환: 조향각 (deg), ±20도 제한
        """
        target_x, target_y = target_point

        lookahead_distance = (target_x**2 + target_y**2) ** 0.5
        if lookahead_distance == 0:
            return 0.0

        steer_rad = atan2(2.0 * 1.0 * target_y, lookahead_distance**2)
        steer_deg = degrees(steer_rad)
        steer_deg = max(min(steer_deg, 20.0), -20.0)

        return steer_deg


    
    def create_motor_command(self, steer_angle, speed):
        """모터 제어 명령 생성"""
        motor_msg = XycarMotor()
        motor_msg.header = Header()
        motor_msg.header.stamp = rospy.Time.now()
        motor_msg.header.frame_id = "map"
        motor_msg.angle = int(steer_angle)
        motor_msg.speed = int(speed)
        
        return motor_msg
    
    
    
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