from time import sleep
import numpy as np
#from .cubic_spline_planner import calc_spline_course
from math import atan2,sin,cos,pi,degrees
import threading

class Motion():
    def __init__(self, sh, pl, eg):
        self.shared = sh
        self.ego = eg
        self.plan = pl


######################속도 기반 정지 보정 함수######################

    def target_control(self, speed):
        self.ego.target_speed = int(speed)

    def stop(self):
        self.target_control(-1)