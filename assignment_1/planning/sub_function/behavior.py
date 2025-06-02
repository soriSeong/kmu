from time import sleep

class Behavior():
    # pc: 인식 정보(신호등 감지, 장애물 등)
    # pl: 계획 모듈(현재 의사결정)
    def __init__(self, sh, eg, pc, pl):
        self.perception = pc
        self.plan = pl
        # 신호등에서 허용된 방향 정보 (빨강 초록 노랑)
        self.traffic_light = False

    def go(self):
        self.plan.motion_decision = "go"

    def stop(self):
        self.plan.motion_decision = "stop"

    def traffic_light(self):
        traffic_state = self.perception.traffic_light
        if traffic_state == False:
            self.plan.motion_decision = "stop"
        elif traffic_state == True:
            self.plan.motion_decision = "go"
        else:
            self.plan.motion_decision = "go"

    def obs_small(self):
        self.plan.motion_decision = "obs_small"

    def obs_big(self):
        self.plan.motion_decision = "obs_big"