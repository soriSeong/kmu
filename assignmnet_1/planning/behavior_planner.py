#!/usr/bin/env python3
import threading
from time import sleep
from .sub_function.behavior import Behavior

class BehaviorPlanner(threading.Thread):
    def __init__(self, parent, rate):
        super().__init__()
        self.period = 1.0 / rate
        
    def run(self):
        while True:
            try:
                if self.plan.behavior_decision == "go":
                    self.behavior.go()
                
                elif self.plan.behavior_decision == "traffic_light":
                    self.behavior.traffic_light()
                # 라바콘
                elif self.plan.behavior_decision == "obs_small":
                    self.behavior.obs_small()
                # 차량 회피
                elif self.plan.behavior_decision == "obs_big":
                    self.behavior.obs_big()

            except IndexError:
                print("++++++++behavior_planner+++++++++")

            sleep(self.period)
