from .ego import Ego
from .perception import Perception
from .path import Path
from .plan import Plan
from .traffic_light import Traffic_light


class Shared:
    def __init__(self):

        self.ego = Ego()

        # for perception
        self.perception = Perception()

        # for planner
        self.global_path = Path()  
        self.local_path = Path()
        self.plan = Plan()

        # for traffic light
        self.traffic_light = False

        # 보조 로직 라바콘 탈출 후 사용
        self.cone_exit_done = False
