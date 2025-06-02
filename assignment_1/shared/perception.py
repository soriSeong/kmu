import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Bool
from math import hypot
import threading
from time import sleep

class Perception:
    def __init__(self):
        self.traffic_light = True
        # 중앙 라인                
        self.lines = []
        # 라바콘 회피 주행에 사용 - 단순한 좌표 리스트로 변경
        self.middle_path = []
        # 라바콘 감지
        self.cone_list = []
        # 차량 감지
        self.vehicle_list = []

        rospy.Subscriber("/traffic_sign",Bool,self.traffic_light_callback)

        rospy.Subscriber("/mid_line_marker_array", MarkerArray, self.middle_path_callback)
        rospy.Subscriber("/obs_small", MarkerArray, self.cone_callback)
        rospy.Subscriber("/static_markers", MarkerArray, self.vehicle_callback)
        self.traffic_light_lock = threading.Lock()
    
    def traffic_light_callback(self, msg):
        self.traffic_light_lock.acquire()
        if msg.data == False:
            self.traffic_light = False
        elif msg.data == True:
            self.traffic_light = True
        else:
            self.traffic_light = True

        self.traffic_light_lock.release()

    def cone_callback(self, msg):
        """라바콘 감지 콜백"""
        self.cone_list = msg.markers
        rospy.logdebug(f"Cone detected: {len(self.cone_list)}개")

    def vehicle_callback(self, msg):
        """차량 감지 콜백"""
        self.vehicle_list = msg.markers
        rospy.logdebug(f"Vehicle detected: {len(self.vehicle_list)}개")

    def middle_path_callback(self, data):
        """중앙 경로 콜백 - MarkerArray에서 좌표 추출"""
        self.middle_path = []
        
        for marker in data.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            self.middle_path.append((x, y))


