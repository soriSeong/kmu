import time

class MissionPlanner(threading.Thread):
    def __init__(self, parent, rate):
        super().__init__()
        self.period = 1.0 / rate
        self.plan = parent.plan
        self.perception = parent.shared.perception

        self.passed_small_obstacle = False  # 작은 장애물 통과 여부 상태

    def run(self):
        while True:
            try:
                # 1. 신호등
                if self.perception.traffic_light == "False":
                    self.plan.mission_decision = "traffic_light"

                # 2. 아직 작은 장애물 안 지났으면
                elif not self.passed_small_obstacle and len(self.perception.obs_list) > 0:
                    self.plan.mission_decision = "obs_small"
                
                # 3. 작은 장애물을 지나고 난 후 큰 장애물 판단
                elif self.passed_small_obstacle and len(self.perception.obs_list) > 0:
                    self.plan.mission_decision = "obs_big"

                # 4. 장애물 인식이 안되면 차선 기반 주행
                else:
                    self.plan.mission_decision = "go"

                # obs_small 감지될 때 시간 저장
                if self.plan.mission_decision == "obs_small":
                    self.last_small_obs_time = time.time()

                # 일정 시간 동안 장애물이 안 보이면 통과했다고 간주 이 후 큰 장애물 회피 주행 준비비
                if (self.plan.mission_decision == "go" and 
                    time.time() - self.last_small_obs_time < 2.0 and
                    len(self.perception.obs_list) == 0):
                    self.passed_small_obstacle = True

            except Exception as e:
                print(f"Exception: {e}")

            time.sleep(self.period)
