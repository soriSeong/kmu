import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Bool
from math import hypot
import threading
from time import sleep

class Perception:
    def __init__(self):
        self.traffic_light = False
        # 중앙 라인                
        self.lines = []
        self.obs_list = []
        # 라바콘 회피 주행에 사용
        self.middle_path = []
        rospy.Subscriber("/traffic_sign",Bool,self.traffic_light_callback)
        rospy.Subscriber("/obs_detection", MarkerArray, self.obs_callback)
        rospy.Subscriber("/middle_path", MarkerArray, self.middle_path_callback)
        self.traffic_light_lock = threading.Lock()
    
    def traffic_light_callback(self, msg):
        self.traffic_light_lock.acquire()
        if msg.data == False:
            self.traffic_light = False
        elif msg.data == True:
            self.traffic_light = True
        else:
            pass

        self.traffic_light_lock.release()

    def obs_callback(self, msg):
        self.obs_list = msg.markers

    def middle_path_callback(self, data):
        self.middle_path = []
        for marker in data.markers:
            if marker.type == Marker.LINE_STRIP:
                lane_points = [(p.x, p.y) for p in marker.points]
                self.middle_path.append(lane_points)
