import math
import numpy as np
import matplotlib.pyplot as plt
import pygame
import rospy
from xycar_msgs.msg import XycarMotor
import control.pure_pursuit as pl

from planning.reeds_shepp_path import (
    calc_optimal_path,
    pi_2_pi,
    STEP_SIZE
)

motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)
xycar_msg = XycarMotor()

class Parking:
    def __init__(self, wheel_base=0.25, lookahead_distance=0.3, max_steer_angle=math.radians(30)):
        self.wheel_base = wheel_base
        self.max_steer_angle = max_steer_angle
        
        # Reeds-Shepp 경로 계획 객체 생성
        self.path_planner = ReedsSheppPath()
        
        # Pure Pursuit 경로 추종 객체 생성
        self.path_follower = PurePursuit(lookahead_distance, wheel_base)
        
        # 현재 경로 저장
        self.current_path = None 
    
    def plan_path(self, start, goal):
        """
        apriltag를 보고 얻은 정보를 토대로 경로 생성
        """
        # 최적 경로 계획
        optimal_path = calc_optimal_path(start, goal, max_curvature, STEP_SIZE)
        
        if optimal_path is None:
            print(" 경로를 찾지 못했습니다.")
            return None
        
        # 경로 저장
        self.current_path = optimal_path
        
        return optimal_path

