import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Int16
from math import hypot
import threading
from time import sleep

class Perception:
    def __init__(self):
        self.obs_list = []
        self.traffic_light = ""

        rospy.Subscriber("/traffic_light",Int16,self.traffic_light_callback)
        rospy.Subscriber("/perception/obstacles", MarkerArray, self.obs_callback)

        self.traffic_light_lock = threading.Lock()
    
    def traffic_light_callback(self, msg):
        self.traffic_light_lock.acquire()
        if msg.data == 0:
            self.traffic_light = "red"
        elif msg.data == 1:
            self.traffic_light = "yellow"
        elif msg.data == 2:
            self.traffic_light = "green"
        else:
            pass

        self.traffic_light_lock.release()

    def obs_callback(self, data):
        self.obs_list = []
        for marker in data.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            width = marker.scale.y
            length = marker.scale.x
            self.obs_list.append([[x,y],width,length])
