#!/usr/bin/env python3
import rospy
import numpy as np
from xycar_msgs.msg import XycarMotor
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from math import atan2, degrees, cos, sin

"""
라이다에서 넘겨주는 토픽 정보를 토대로 회피 주행을 수행한다.
"""
class ConeDrivingController:
    def __init__(self, shared):
        """
        생성자
        - shared 객체를 통해 ego(차량 상태), perception(주변 정보)를 받아옴
        - 차량 휠베이스(wheelbase)는 1.0m로 설정
        - 라바콘 경로를 시각화하기 위한 Publisher를 '/cone_path' 토픽에 생성
        """
        self.shared = shared
        self.ego = shared.ego
        self.perception = shared.perception

        self.wheelbase = 1.0  # 휠베이스

        self.path_pub = rospy.Publisher('/cone_path', Path, queue_size=1)

        rospy.loginfo("Cone Driving Controller initialized")

    def follow_cone_path(self):
        """
        라바콘 경로 추종 메인 함수
        perception.middle_path가 비어있으면 경고 로그를 남기고 리턴한다.
        경로상 가장 앞쪽 지점을 선택 (리스트 인덱스 2번 or 마지막 인덱스)
        절대 좌표로부터 차량 기준 상대 좌표로 변환
        Pure Pursuit 알고리즘으로 조향각계산
        생성된 조향각과 속도로 XycarMotor 메시지 생성 후 리턴한다.
        예외 상황 발생 시 에러 로그를 남기고 직진한다.
        """
        try:
            if not self.perception.middle_path:
                rospy.logwarn("No middle path available")
                return None

            abs_target = self.perception.middle_path[min(len(self.perception.middle_path) - 1, 2)]

            target_point = self.to_relative(abs_target[0], abs_target[1], self.ego)

            steer_angle = self.pure_pursuit_from_relative_point(target_point)
            steer_cmd = steer_angle

            target_speed = 5.5

            return self.create_motor_command(steer_cmd, target_speed)

        except Exception as e:
            rospy.logerr(f"following Cone path error: {e}")
            return self.create_motor_command(0, 10)
    
    def to_relative(self, x, y, ego):
        """
        절대좌표를 차량 기준 상대좌표로 변환
        - x, y: 맵 좌표계(절대좌표)상의 목표 지점
        - ego: 차량의 현재 위치(x, y)와 yaw
        - rel_x, rel_y: 차량 전방 기준으로 바라봤을 때의 상대좌표
        cos, sin 사용해 차량 yaw 각도를 이용해 회전 변환한다.
        회전 변환 결과가 차량 전방 기준 상대좌표로 변환된 값이다.
        """
        dx = x - ego.x
        dy = y - ego.y
        yaw = ego.yaw

        rel_x = cos(-yaw) * dx - sin(-yaw) * dy
        rel_y = sin(-yaw) * dx + cos(-yaw) * dy
        return rel_x, rel_y

    def pure_pursuit_from_relative_point(self, target_point):
        """
        Pure Pursuit 알고리즘으로 조향각 계산 : 차량 기준 상대좌표
        - target_point: (rel_x, rel_y), 차량 전방 기준 상대좌표
        - steer_deg: 조향각 (degrees), 좌우 최대 ±20도로 제한
        lookahead_distance = sqrt(rel_x^2 + rel_y^2)
            - 목표 지점까지의 직선 거리
        lookahead_distance가 0이면 조향각을 0으로 설정
        Pure Pursuit 공식: steer_rad = atan2(2 * L * y, d^2)을 사용한다.
            - L: 휠베이스
            - y: 상대 좌표 y (차량 전방 기준 좌우 오프셋)
            - d: lookahead_distance
        steer_rad(라디안) → steer_deg(도) 변환한다.
        steer_deg를 최대 ±20도로 제한하여 리턴한다.
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
        """
        XycarMotor 메시지 생성 및 리턴
        frame_id는 "map"으로 설정
        angle은 xycar 환경에 맞게 설정했다.
        """
        motor_msg = XycarMotor()
        motor_msg.header = Header()
        motor_msg.header.stamp = rospy.Time.now()
        motor_msg.header.frame_id = "map"
        motor_msg.angle = int(steer_angle * 5)
        motor_msg.speed = speed
        return motor_msg

    def get_status(self):
        """
        현재 컨트롤러 상태 정보 리턴
        """
        return {
            'middle_path_length': len(self.perception.middle_path) if self.perception.middle_path else 0,
            'lookahead_distance': 2.0,
            'max_steer_angle': 20.0
        }
