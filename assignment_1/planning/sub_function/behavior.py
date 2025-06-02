

class Behavior():
    """
    행동 결정을 담당하는 클래스
    pc: 인식 정보(신호등 감지, 장애물 등)
    pl: 계획 모듈(현재 의사결정)
    traffic_light: 신호등 상태 true면 직전(초록불)
    obs_small: 작은 장애물(라바콘 회피)
    obs_big: 큰 장애물(차량 회피)
    """
    def __init__(self, pc, pl):
        self.perception = pc
        self.plan = pl

    def go(self):
        self.plan.motion_decision = "go"
        
    def traffic_light_decision(self):
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