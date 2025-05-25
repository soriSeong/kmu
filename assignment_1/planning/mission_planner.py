import time
import threading

class MissionPlanner(threading.Thread):
    def __init__(self, parent, rate):
        super().__init__()
        self.period = 1.0 / rate
        self.plan = parent.plan
        self.perception = parent.shared.perception

        self.passed_small_obstacle = False # 작은 장애물 통과 여부 상태
        self.last_small_obs_time = 0.0 # 작은 장애물 감지 시간

    def run(self):
        while True:
            try:
                # 신호등 미션 - False일 때만 정지
                if self.perception.traffic_light == False:
                    self.plan.mission_decision = "traffic_light"
                    self.plan.behavior_decision = "traffic_light"  # behavior에 전달
                    self.plan.motion_decision = "traffic_light"    # motion에 전달

                # 아직 작은 장애물 안 지났으면 작은 장애물이라 판단 후 라이다에서 넘겨주는 토픽으로 회피 할 준비
                elif not self.passed_small_obstacle and len(self.perception.obs_list) > 0:
                    self.plan.mission_decision = "obs_small"
                    self.plan.behavior_decision = "obs_small"
                    self.plan.motion_decision = "obs_small"
                
                # 작은 장애물을 지나고 난 후 큰 장애물 판단 optimal frenet을 활용해서 차량 회피
                elif self.passed_small_obstacle and len(self.perception.obs_list) > 0:
                    self.plan.mission_decision = "obs_big"
                    self.plan.behavior_decision = "obs_big"
                    self.plan.motion_decision = "obs_big"

                # 장애물 인식이 안되면 차선 기반 주행 계속 실행
                else:
                    self.plan.mission_decision = "go"
                    self.plan.behavior_decision = "go"
                    self.plan.motion_decision = "go"

                # obs_small 감지될 때 시간 저장
                if self.plan.mission_decision == "obs_small":
                    self.last_small_obs_time = time.time()

                # 일정 시간 동안 장애물이 안 보이면 통과했다고 간주 이 후 큰 장애물 회피 주행 준비
                if (self.plan.mission_decision == "go" and 
                    time.time() - self.last_small_obs_time < 2.0 and
                    len(self.perception.obs_list) == 0):
                    self.passed_small_obstacle = True
                    print("Small obstacle passed!")

                # 디버깅 정보 출력
                print(f"Status: Traffic={self.perception.traffic_light}, Obs={len(self.perception.obs_list)}, Decision={self.plan.mission_decision}")

            except Exception as e:
                print(f"Mission Planner Exception: {e}")
                import traceback
                traceback.print_exc()

            time.sleep(self.period)