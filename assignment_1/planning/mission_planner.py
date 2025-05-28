import time
import threading
import rospy

class MissionPlanner(threading.Thread):
    def __init__(self, parent, rate):
        super().__init__()
        self.period = 1.0 / rate
        self.plan = parent.plan
        self.perception = parent.shared.perception
        
    def run(self):
        while True:
            try:
                # 신호등 미션 - False일 때만 정지
                if self.perception.traffic_light == False:
                    self.plan.mission_decision = "traffic_light"
                    self.plan.behavior_decision = "traffic_light"  # behavior에 전달
                    self.plan.motion_decision = "traffic_light"    # motion에 전달

                # 라바콘 회피 (콘이 감지되면)
                elif len(self.perception.cone_list) > 0:
                    self.plan.mission_decision = "obs_small"
                    self.plan.behavior_decision = "obs_small"
                    self.plan.motion_decision = "obs_small"

                # 차량 회피 (차량이 감지되면)
                elif len(self.perception.vehicle_list) > 0:
                    self.plan.mission_decision = "obs_big"
                    self.plan.behavior_decision = "obs_big"
                    self.plan.motion_decision = "obs_big"

                # 장애물이 없으면 일반 주행
                else:
                    self.plan.mission_decision = "go"
                    self.plan.behavior_decision = "go"
                    self.plan.motion_decision = "go"
            except Exception as e:
                print(f"Mission Planner Exception: {e}")
                import traceback
                traceback.print_exc()

            time.sleep(self.period)