#!/usr/bin/env python3
import rospy
import numpy as np
from xycar_msgs.msg import XycarMotor
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from math import atan2, degrees, cos, sin


class ConeDrivingController:
    def __init__(self, shared):
        self.shared = shared
        self.ego = shared.ego
        self.perception = shared.perception

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

            # 경로상 가장 앞쪽점을 추종하도록 설정
            abs_target = self.perception.middle_path[min(len(self.perception.middle_path) - 1, 2)]

            # 절대좌표 → 차량 기준 상대좌표 변환
            target_point = self.to_relative(abs_target[0], abs_target[1], self.ego)

            # 조향각 계산
            steer_angle = self.pure_pursuit_from_relative_point(target_point)
            steer_cmd = steer_angle # 스케일 조정

            # 속도 설정
            target_speed = 5.5

            return self.create_motor_command(steer_cmd, target_speed)

        except Exception as e:
            rospy.logerr(f"Cone path following error: {e}")
            return self.create_motor_command(0, 10)

    def to_relative(self, x, y, ego):
        """절대좌표를 차량 기준 상대좌표로 변환"""
        dx = x - ego.x
        dy = y - ego.y
        yaw = ego.yaw  # 라디안 단위

        rel_x = cos(-yaw) * dx - sin(-yaw) * dy
        rel_y = sin(-yaw) * dx + cos(-yaw) * dy
        return rel_x, rel_y

    def pure_pursuit_from_relative_point(self, target_point):
        """
        Pure Pursuit 방식으로 조향각 계산 (차량 기준 상대좌표 입력)
        """
        target_x, target_y = target_point
        lookahead_distance = (target_x ** 2 + target_y ** 2) ** 0.5

        if lookahead_distance == 0:
            return 0.0

        steer_rad = atan2(2.0 * self.wheelbase * target_y, lookahead_distance ** 2)
        steer_deg = degrees(steer_rad)
        steer_deg = max(min(steer_deg, 20.0), -20.0)

        return steer_deg

    def create_motor_command(self, steer_angle, speed):
        """XycarMotor 메시지 생성 및 반환"""
        motor_msg = XycarMotor()
        motor_msg.header = Header()
        motor_msg.header.stamp = rospy.Time.now()
        motor_msg.header.frame_id = "map"
        motor_msg.angle = int(steer_angle*5)
        motor_msg.speed = speed
        return motor_msg

    def emergency_stop(self):
        """비상 정지 명령 생성"""
        rospy.logwarn("Cone driving emergency stop!")
        return self.create_motor_command(0, 0)

    def get_status(self):
        """상태 정보 반환"""
        return {
            'middle_path_length': len(self.perception.middle_path) if self.perception.middle_path else 0,
            'lookahead_distance': 2.0,
            'max_steer_angle': 20.0
        }
