from .ego import Ego
from .perception import Perception_
from .path import Path
from .plan import Plan
from .traffic_light import Traffic_light
from .right_turn import Right_turn


class Shared:
    def __init__(self):
        
        # in common
        #self.ego = Ego()

        # for perception
        self.perception = Perception_()

        # for planner
        self.global_path = Path()  
        self.local_path = Path()
        self.plan = Plan()

        # for traffic light
        self.traffic_light = Traffic_light()
        
        # for right turn
        self.right_turn = Right_turn()

        # for obstacle avoidance
        self.obstacles = []
        self.dangerous_obstacles = []

        # for parallel parking
        self.pp_final = False
        self.parallel_local_control = False

        # for self control steer
        self.manual_steer = False

        # for obstacle avoidance ending optimization
        self.obs_ending_flag = False

