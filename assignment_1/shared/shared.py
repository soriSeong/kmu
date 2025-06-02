from .ego import Ego
from .perception import Perception
from .plan import Plan


class Shared:
    """
    공유되는 상태 객체
    """
    def __init__(self):

        self.ego = Ego()

        # for perception
        self.perception = Perception()

        # for planner
        self.plan = Plan()

