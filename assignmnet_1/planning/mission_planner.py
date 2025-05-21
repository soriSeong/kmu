import threading
from time import sleep

class MissionPlanner(threading.Thread):
    def __init__(self, parent, rate):
        super().__init__()
        self.period = 1.0 / rate
        self.plan = parent.plan

    def run(self):
        while True:
            try:
                if self.plan.mission_decision == "go":
                    self.plan.behavior_decision = "go"
                
                elif self.plan.mission_decision == "traffic_light":
                    self.plan.behavior_decision = "traffic_light"
                # 라바콘
                elif self.plan.mission_decision == "obs_small":
                    self.plan.behavior_decision = "obs_small"
                # 차량 회피
                elif self.plan.mission_decision == "obs_big":
                    self.plan.behavior_decision = "obs_big"

            except IndexError:
                print("++++++++mission_planner+++++++++")

            sleep(self.period)
