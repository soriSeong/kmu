#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Bool
from math import hypot
import threading
from time import sleep

class Perception:
    """
    센서 데이터 수신 및 전처리를 담당하는 클래스
    - 라이다에서 MarkerArray 기반으로 라바콘(작은 장애물)과 차량(큰 장애물)을 감지한다.
    - 카메라에서 보낸 토픽을 Bool 타입("/traffic_sign")으로 수신한다.
    - 감지된 정보는 Shared.perception을 통해 전달한다.
    """
    def __init__(self):
        """
        생성자
        1. traffic_light 초기화(True: 초록불)한다.
        2. 라바콘 회피 주행에 사용될 middle_path 초기화(empty list)한다.
        3. cone_list, vehicle_list 초기화한다.
        4. "/traffic_sign"(Bool) Subscriber
        5. "/mid_line_marker_array"(MarkerArray) Subscriber
        6. "/obs_small"(MarkerArray) Subscriber
        7. "/static_markers"(MarkerArray) Subscriber
        8. 값이 일관성 있게 유지되도록 traffic_light_lock을 생성한다.
        """
        self.traffic_light = True
        self.middle_path = []
        self.cone_list = []
        self.vehicle_list = []
        rospy.Subscriber("/traffic_sign", Bool, self.traffic_light_callback)
        rospy.Subscriber("/mid_line_marker_array", MarkerArray, self.middle_path_callback)
        rospy.Subscriber("/obs_small", MarkerArray, self.cone_callback)
        rospy.Subscriber("/static_markers", MarkerArray, self.vehicle_callback)
        self.traffic_light_lock = threading.Lock()
    
    def traffic_light_callback(self, msg):
        """
        신호등 상태 콜백 함수
        Bool 타입 초록불 True, 빨간불 or 노란불 False
        1. traffic_light_lock 획득
        2. msg.data가 False면 self.traffic_light=False, True면 self.traffic_light=True
        3. lock 해제
        """
        self.traffic_light_lock.acquire()
        if msg.data == False:
            self.traffic_light = False
        elif msg.data == True:
            self.traffic_light = True
        else:
            self.traffic_light = True
        self.traffic_light_lock.release()

    def cone_callback(self, msg):
        """
        라바콘 감지 콜백 함수
        MarkerArray 타입
        1. self.cone_list에 msg.markers 저장
        2. rospy.logdebug로 감지된 라바콘 개수 출력
        """
        self.cone_list = msg.markers
        rospy.logdebug(f"Cone detected: {len(self.cone_list)}개")

    def vehicle_callback(self, msg):
        """
        차량 감지 콜백 함수
        MarkerArray 타입
        1. self.vehicle_list에 msg.markers 저장
        2. rospy.logdebug로 감지된 차량 개수 출력
        """
        self.vehicle_list = msg.markers
        rospy.logdebug(f"Vehicle detected: {len(self.vehicle_list)}개")

    def middle_path_callback(self, data):
        """
        MarkerArray에서 좌표 추출
        1. self.middle_path를 빈 리스트로 초기화한다.
        2. 각marker에 대해 x, y 좌표를 추출한다.
        3. (x, y) 튜플 형태로 self.middle_path에 append한다.
        """
        self.middle_path = []
        for marker in data.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            self.middle_path.append((x, y))
