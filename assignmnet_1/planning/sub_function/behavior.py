from time import sleep

class Behavior():
    # sh: 공유 정보(지도, 주행 상태 등)
    # eg: 차량의 상태
    # pc: 인식 정보(신호등 감지, 장애물 등)
    # pl: 계획 모듈(현재 의사결정 등)
    def __init__(self, sh, eg, pc, pl):
        self.perception = pc
        self.shared = sh
        self.ego = eg
        self.plan = pl
        self.obs_decision()
        # 신호등에서 허용된 방향 정보 (빨강 초록 노랑)
        self.traffic_light = "red"

    def go(self):
        self.plan.motion_decision = "go"

    def stop(self):
        self.plan.motion_decision = "stop"

    def traffic_light(self):
        signal_state = self.perception.traffic_light_state
        if signal_state == "red" or signal_state == "yellow":
            self.plan.behavior_decision = "stop"
        elif signal_state == "green":
            self.plan.behavior_decision = "go"

    # obs 사이즈에 따라 obs small obs big 결정
    def obs_decision(self):
        obss = self.perception.obs_list
        if len(obss) == 0:
            self.plan.motion_decision = "go"
            return

        for obs in obss:
            _, width, length = obs

            if width > 1.5 or length > 2.0:  # 임계값 기준 # 시뮬 돌리면서 테스트 해야됨
                self.plan.motion_decision = "obs_big"
                return

        # 모든 장애물이 작으면
        self.plan.motion_decision = "obs_small"


