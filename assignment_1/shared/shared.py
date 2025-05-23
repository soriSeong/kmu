from .ego import Ego
from .perception import Perception
from .path import Path
from .plan import Plan
from .traffic_light import Traffic_light


class Shared:
    def __init__(self):

        # for perception
        self.perception = Perception()

        # for planner
        self.global_path = Path()  
        self.local_path = Path()
        self.plan = Plan()

        # for traffic light
        self.traffic_light = Traffic_light()

        # for obstacle avoidance
        self.obstacles = []
        self.dangerous_obstacles = []