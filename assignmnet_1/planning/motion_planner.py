import threading
from time import sleep
from .sub_function.motion import Motion
from numpy import dot
from math import cos, sin

class MotionPlanner(threading.Thread):
    def __init__(self, parent, rate):
        super().__init__()
        self.period = 1.0 / rate
        self.motion = Motion()
    
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