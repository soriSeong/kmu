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

        # 신호등에서 허용된 방향 정보 (빨강 초록 노랑)
        self.pass_signal_list = []

    def go(self):
        self.plan.motion_decision = "go"

    def stop(self):
        self.plan.motion_decision = "stop"

    def traffic_light(self):
        signal_state = self.perception.traffic_light_state
        if signal_state == "red":
            self.plan.behavior_decision = "stop"
        elif signal_state == "yellow": # 출발 시 신호등 정보에 따라야하기 때문에 정지 후 출발 = 정지임임
            self.plan.behavior_decision = "stop"
        elif signal_state == "green":
            self.plan.behavior_decision = "go"

    # 라바콘 회피 주행
    # 라이다 관린 정보로 control해서 라바콘 회피피
    def obs_small(self):
        self.plan.motion_decision = "obs_small"

    def obs_big(self):
        # 차량 회피 optimal frenet planning
        pass
