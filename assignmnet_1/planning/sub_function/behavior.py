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
        self.traffic_light = False

    def go(self):
        self.plan.motion_decision = "go"

    def stop(self):
        self.plan.motion_decision = "stop"

    def traffic_light(self):
        if self.traffic_light == False:
            self.plan.behavior_decision = "stop"
        elif self.traffic_light == True:
            self.plan.behavior_decision = "go"