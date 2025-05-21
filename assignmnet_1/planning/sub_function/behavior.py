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

    def go(self):
        self.plan.motion_decision = "go"

    def traffic_light(self):
        pass

    def obs_small(self):
        pass

    def obs_big(self):
        # 차량 회피 optimal frenet planning
        pass
