import threading
from time import sleep
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf
from .sub_function.motion import Motion
from controller.lane_driving import LaneDrivingController
from .optimal_frenet import *
import rospy


class MotionPlanner(threading.Thread):
    def __init__(self, parent, rate):
        super().__init__()
        self.period = 1.0 / rate
        self.shared = parent.shared
        self.plan = parent.shared.plan
        self.ego = parent.shared.ego 
        self.motion = Motion(self.shared)
        self.path_pub = rospy.Publisher("/frenet_path", Path, queue_size=1)
    
        self.lane_controller = LaneDrivingController(self.shared)

    def run(self):
        while True:
            try:
                if self.plan.motion_decision == "go":
                    self.motion.go()
                
                elif self.plan.motion_decision == "traffic_light":
                    self.motion.traffic_light()
                # 라바콘
                elif self.plan.motion_decision == "obs_small":
                    self.motion.obs_small()
                # 차량 회피
                elif self.plan.motion_decision == "obs_big":
                    self.motion.obs_big()

            except IndexError:
                print("++++++++motion_planner+++++++++")

            sleep(self.period)