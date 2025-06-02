from .path import Path

class Plan():
    def __init__(self):
        # 현재 미션 정보
        self.mission_decision = ""
        self.behavior_decision = ""
        self.motion_decision = ""