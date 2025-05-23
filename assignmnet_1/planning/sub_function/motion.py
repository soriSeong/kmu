import rospy
from xycar_msgs.msg import XycarMotor
from frenet_controller import FrenetController
from time import sleep
import numpy as np

class Motion():
    def __init__(self, sh, pl, eg):
        self.shared = sh
        self.plan = pl
        self.ego = eg
        self.controller = FrenetController()
        self.actuator_pub = rospy.Publisher('/xycar_motor', XycarMotor, queue_size=10)
        
    def target_control(self, speed):
        self.ego.target_speed = int(speed)

    def stop(self):
        self.target_control(-1)

    def go(self):
        self.target_control(30)

    def traffic_light(self):
        traffic_state = self.shared.traffic_light.state
        if traffic_state == "red" or traffic_state == "yellow":
            self.stop()
        elif traffic_state == "green":
            self.target_control(30)

    def obs_small(self):
        self.target_control(20)

    # 제어 담당
    def obs_big(self):
        """차량 회피 - Optimal Frenet + 제어 실행"""
        path = self.shared.local_path
        if not path or not hasattr(path, 'x') or len(path.x) < 2:
            rospy.logwarn("경로 없음 또는 너무 짧음")
            self.target_control(0)
            return

        msgs = self.controller.follow_frenet_path(
            path,
            ego_x=self.ego.x,
            ego_y=self.ego.y,
            yaw=self.ego.yaw,
            speed=self.ego.speed,
            steer_prev=self.ego.curr_steer
        )

        if msgs:
            for msg in msgs:
                self.actuator_pub.publish(msg)
        else:
            rospy.logwarn("유효한 제어 메시지 없음")
            self.target_control(0)
