import numpy as np
from math import atan2, sin, cos, degrees, sqrt
from std_msgs.msg import Header
from xycar_msgs.msg import XycarMotor
from numpy.linalg import norm
import rospy

LOOKAHEAD_OFFSET = 3

class PurePursuit:
    def __init__(self):
        self.L = 3
        self.k = 0.14
        self.Lfc = 6.0
        self.alpha = 1.5

    def euc_distance(self, pt1, pt2):
        return norm([pt2[0] - pt1[0], pt2[1] - pt1[1]])

    def global_to_local(self, target_point, position, yaw):
        dx = target_point[0] - position[0]
        dy = target_point[1] - position[1]
        x_local = dx * cos(-yaw) - dy * sin(-yaw)
        y_local = dx * sin(-yaw) + dy * cos(-yaw)
        return x_local, y_local

    def vel_global_to_local(self, target_point, position, yaw):
        return self.global_to_local(target_point, position, yaw)

    def run(self, vEgo, target_point, position, yaw, sEgo):
        lfd = self.Lfc + self.k * vEgo
        lfd = np.clip(lfd, 5, 10)
        x_local , y_local = self.vel_global_to_local(target_point, position, yaw)
        diff = np.sqrt(x_local**2 + y_local**2)
        if diff > 0:
            if diff >= lfd:
                theta = atan2(y_local, x_local)
                steering_angle = atan2(2 * self.L * sin(theta), lfd)
                return degrees(steering_angle), target_point
        return 0.0, target_point

    def run_global(self, vEgo, target_point, position, yaw, sEgo):
        lfd = self.Lfc + self.k * vEgo
        lfd = np.clip(lfd, 5, 10)
        x_local , y_local = self.global_to_local(target_point, position, yaw)
        diff = np.sqrt(x_local**2 + y_local**2)
        if diff > 0:
            if diff >= lfd:
                theta = atan2(y_local, x_local)
                steering_angle = atan2(2 * self.L * sin(theta), lfd)
                return degrees(steering_angle), target_point
        return 0.0, target_point

class PID:
    def __init__(self, kp, ki, kd, dt=0.05):
        self.K_P = kp
        self.K_I = ki
        self.K_D = kd
        self.pre_error = 0.0
        self.integral_error = 0.0
        self.dt = dt

    def run(self, target, current):
        error = sqrt((target[0] - current[0])**2 + (target[1] - current[1])**2)
        derivative_error = (error - self.pre_error) / self.dt
        self.integral_error += error * self.dt
        self.integral_error = np.clip(self.integral_error, -5, 5)
        pid = self.K_P * error + self.K_I * self.integral_error + self.K_D * derivative_error
        pid = np.clip(pid, -100, 100)
        self.pre_error = error
        return pid

class FrenetController:
    def __init__(self, steer_ratio=12):
        self.pure_pursuit = PurePursuit()
        self.pid = PID(kp=1.0, ki=0.1, kd=0.01)
        self.steer_ratio = steer_ratio
        self.inter_steer = 0.0

    def get_closest_waypoints(self, x, y, path_x, path_y):
        min_dist = float('inf')
        closest_idx = 0
        for i in range(len(path_x)):
            dist = np.sqrt((x - path_x[i])**2 + (y - path_y[i])**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        return closest_idx

    def follow_frenet_path(self, path, ego_x, ego_y, yaw, speed, steer_prev):
        idx = self.get_closest_waypoints(ego_x, ego_y, path.x, path.y)
        if idx >= len(path.x) - 1:
            return None  # 종료 조건

        tgt_idx = min(idx + LOOKAHEAD_OFFSET, len(path.x) - 1)
        target_point = (path.x[tgt_idx], path.y[tgt_idx])
        position = (ego_x, ego_y)

        steer_deg, target = self.pure_pursuit.run_global(speed, target_point, position, yaw, steer_prev)
        steer = steer_deg * self.steer_ratio
        throttle = self.pid.run(target, position)
        throttle = np.clip(throttle, 0, 10)

        steer_interp = np.linspace(self.inter_steer, steer, 10)
        xycars = []
        for s in steer_interp:
            a = XycarMotor()
            a.header = Header()
            a.header.stamp = rospy.Time.now()
            a.header.frame_id = "map"
            a.angle = s
            a.speed = throttle
            xycars.append(a)

        self.inter_steer = steer
        return xycars