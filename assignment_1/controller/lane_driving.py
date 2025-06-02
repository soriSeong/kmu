#!/usr/bin/env python3

import numpy as np
from math import atan2, sin, cos, degrees, sqrt
from std_msgs.msg import Header, Int32, String, Float32
from xycar_msgs.msg import XycarMotor
from geometry_msgs.msg import PoseStamped, TwistStamped
from numpy.linalg import norm
import rospy
import tf

# 파라미터 설정
TARGET_SPEED = 35  # 목표 속도
IMAGE_WIDTH = 640  # 이미지 너비
IMAGE_CENTER = 320  # 이미지 중심

class LateralPIDRelative:
    """
    생성자
    - kp, ki, kd: PID 설정
    - dt: 제어 주기
    - previous_error: 이전 에러 값 초기화
    - integral: 적분 값 초기화
    - error_history: 최근 픽셀 오차를 저장해 이동평균 처리
    """
    def __init__(self, kp=1.5, ki=0.05, kd=0.2, dt=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.previous_error = 0.0
        self.integral = 0.0
        self.error_history = []


    def update_from_pixel_error(self, target_x, current_x=IMAGE_CENTER, image_width=IMAGE_WIDTH):
        """
        차선 중심 픽셀 오차를 이동평균으로 부드럽게 처리 
        픽셀 오차를 약 0.01m/px로 환산하여 횡방향 거리 오차로 변환하고 데드존(+-0.2m) 적용,
        PID 제어기를 통해 최종 조향 값을 [-20, 20]도로 계산한다.
        """
        pixel_error = target_x - current_x
        
        self.error_history.append(pixel_error)
        if len(self.error_history) > 5:
            self.error_history.pop(0)
        
        smoothed_error = sum(self.error_history) / len(self.error_history)

        lateral_distance_error = (smoothed_error / (image_width/4)) * 2.0  # 상대거리
        
        if abs(lateral_distance_error) < 0.2:
            lateral_distance_error *= 0.5
        
        return self.update_lateral_error(lateral_distance_error)

    def update_lateral_error(self, lateral_error):
        """
        상대좌표 기준 횡방향 에러를 받아 PID 제어를 수행하고
        조향 각도(도)를 [-20, 20] 범위로 반환한다.
        """
        derivative = (lateral_error - self.previous_error) / self.dt
        self.integral += lateral_error * self.dt
        self.integral = np.clip(self.integral, -5.0, 5.0)

        output = (self.kp * lateral_error +
                  self.ki * self.integral +
                  self.kd * derivative)

        self.previous_error = lateral_error
        return np.clip(output, -20, 20)

class SpeedPIDRelative:
    """상대좌표 기준 속도 제어용 PID"""

    def __init__(self, kp=1.5, ki=0.05, kd=0.2, dt=0.05):
        """
        PID 속도 제어기의 이득 및 상태 초기화
        - kp, ki, kd: PID 이득
        - dt: 제어 주기
        - previous_v: 이전 속도 오차 값
        - integral: 적분 누적 값
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.previous_v = 0.0
        self.integral = 0.0

    def update_speed_error(self, target_speed, current_speed):
        """
        목표 속도와 현재 속도의 차이를 PID로 제어하여
        스로틀 값을 [0, 40] 범위로 반환한다.
        """
        speed_error = target_speed - current_speed
        
        derivative = (speed_error - self.previous_v) / self.dt
        self.integral += speed_error * self.dt
        self.integral = np.clip(self.integral, -5, 5)

        output = (self.kp * speed_error +
                  self.ki * self.integral +
                  self.kd * derivative)

        self.previous_v = speed_error
        return max(0, np.clip(output, 0, 40))

class LaneDrivingController:
    """상대좌표 기준 차선 주행 제어기"""

    def __init__(self, shared, steer_ratio=1.0):
        """
        LaneDrivingController 생성자
        - shared: ego(차량 상태), perception(차선 인식 정보)을 포함한 Shared 객체
        - steer_ratio: 조향 비율 스케일
        - rospy.Subscriber를 통해 target_x, lane_status, lane_curvature, current_pose, current_velocity를 수신한다.
        - curve_direction는 -1일 시 좌회전, 0 은 직진 1 은 우회전이다. 이미지 정보를 토대로 결정 될 것이다.
        """
        self.shared = shared
        self.ego = shared.ego if hasattr(shared, 'ego') else None
        self.perception = shared.perception

        self.lateral_pid = LateralPIDRelative(kp=1.9, ki=0.03, kd=0.4)
        self.speed_pid = SpeedPIDRelative(kp=1.5, ki=0.05, kd=0.2)
        self.steer_ratio = steer_ratio

        self.prev_steer = 0.0
        self.steer_history = []

        self.target_x = IMAGE_CENTER
        self.lane_status = "BOTH"
        self.lane_curvature = 0.0
        self.curve_direction = 0

        self.target_x_received = False
        self.lane_status_received = False

        rospy.Subscriber('/lane_detection/target_x', Int32, self.target_x_callback)
        rospy.Subscriber('/lane_detection/lane_status', String, self.lane_status_callback)
        rospy.Subscriber('/lane_detection/lane_curvature', Float32, self.lane_curvature_callback)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_callback)

        rospy.loginfo("Lane Driving Controller initialized (Curve Direction Detection)")

    def target_x_callback(self, msg):
        """
        차선 중심 픽셀 좌표를 수신하여 유효 범위 검사 후
        target_x에 저장하고 플래그를 설정한다.
        """
        if 50 <= msg.data <= 590:
            self.target_x = msg.data
            self.target_x_received = True
        else:
            rospy.logwarn(f"Invalid target_x received: {msg.data}")


    def lane_status_callback(self, msg):
        """
        차선 상태(“BOTH”, “LEFT”, “RIGHT”, “LOST”)를 수신하여
        lane_status에 저장하고 플래그를 설정한다.
        """
        if msg.data in ["BOTH", "LEFT", "RIGHT", "LOST"]:
            self.lane_status = msg.data
            self.lane_status_received = True
        else:
            rospy.logwarn(f"Invalid lane_status received: {msg.data}")

    def lane_curvature_callback(self, msg):
        """
        차선 곡률 값을 수신하여 lane_curvature에 저장한다.
        """
        self.lane_curvature = msg.data

    def pose_callback(self, msg):
        """
        차량의 현재 위치와 yaw(오일러 각)를 업데이트한다.
        - msg.pose.position: x, y 좌표(m) 저장
        - msg.pose.orientation: quaternion을 오일러로 변환하여 yaw 저장
        """
        if self.ego:
            self.ego.x = msg.pose.position.x
            self.ego.y = msg.pose.position.y
            q = msg.pose.orientation
            euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
            self.ego.yaw = euler[2]

    def velocity_callback(self, msg):
        """
        차량의 현재 속도를 업데이트한다.
        - msg.twist.linear.x: 속도(m/s)를 ego.speed에 저장
        """
        if self.ego:
            self.ego.speed = msg.twist.linear.x

    def detect_curve_direction(self):
        """
        target_x 위치를 기준으로 커브 방향을 판단한다.
        - target_x < IMAGE_CENTER: -1 (좌회전)
        - target_x > IMAGE_CENTER: 1  (우회전)
        - 그 외: 0 (직진)
        """
        if self.target_x < IMAGE_CENTER:
            return -1
        elif self.target_x > IMAGE_CENTER:
            return 1
        else:
            return 0

    def calculate_target_speed_relative(self):
        """
        차선 곡률(lane_curvature)의 절댓값에 따라 목표 속도를 조정하여 반환한다.
        카메라에서 주어지는 곡률을 활용해 파라미터 튜닝을 진행했다.
        - 곡률 > 1000000: 기본 속도의 0.3배
        - 곡률 > 700000: 기본 속도의 0.4배
        - 곡률 > 500000: 기본 속도의 0.5배
        - 곡률 < 300000: 기본 속도의 0.6배
        - 곡률 < 200000: 기본 속도의 0.7배
        - 그 외: 기본 속도(TARGET_SPEED)
        """
        base_speed = TARGET_SPEED
        curvature_abs = abs(self.lane_curvature)

        if curvature_abs > 1000000:
            return base_speed * 0.3
        if curvature_abs > 700000:
            return base_speed * 0.4
        elif curvature_abs > 500000:
            return base_speed * 0.5
        elif curvature_abs < 300000:
            return base_speed * 0.6
        elif curvature_abs < 200000:
            return base_speed * 0.7
        else:
            return base_speed

    def smooth_steering_relative(self, new_steer):
        """
        최근 계산된 조향값(new_steer)을 저장하고 이동평균으로 smoothing 한 후,
        곡률 기반 최대 변화량을 계산하여 급격한 변화를 제한한 최종 조향값을 반환한다.
        """
        self.steer_history.append(new_steer)
        if len(self.steer_history) > 3:
            self.steer_history.pop(0)

        smoothed = sum(self.steer_history) / len(self.steer_history)
        max_change = 8.0 + abs(self.lane_curvature / 50000.0)
        max_change = np.clip(max_change, 8.0, 20.0)

        if abs(smoothed - self.prev_steer) > max_change:
            smoothed = self.prev_steer + max_change * np.sign(smoothed - self.prev_steer)

        return smoothed

    def lane_following_control(self):
        """
        차선 추종을 위해 현재 속도와 곡률 기반으로 목표 속도를 계산하고,
        lane_status에 따라 PID 조향 및 곡률 보정, LEFT/RIGHT 오프셋을 적용하여
        최종 조향각과 스로틀 계산 후 XycarMotor 메시지를 반환한다.
        np.clip(final_steer, -20, 20)을 사용해 xycar의 특성에서 벗어나지 않도록 한다.
        smooth_steering_relative()에서 이동평균과 최대 변화량 제한으로 급격한 각도 변화를 억제한다.
        """
        ego_speed = self.ego.speed if self.ego else 5
        target_speed = self.calculate_target_speed_relative()

        if not self.target_x_received:
            self.target_x = IMAGE_CENTER
        if not self.lane_status_received:
            self.lane_status = "BOTH"

        self.curve_direction = self.detect_curve_direction()

        if self.lane_status == "LOST":
            steering_angle = self.prev_steer * 0.9
            target_speed *= 0.8
            curvature_gain = 0.0
            pid_steer = 0.0
        else:
            pid_steer = self.lateral_pid.update_from_pixel_error(
                self.target_x, IMAGE_CENTER, IMAGE_WIDTH
            )
            steering_angle = pid_steer

            curvature_gain = 0.0
            if self.lane_curvature > 700000:
                base_gain = self.lane_curvature / 700000.0
                curvature_gain = np.clip(base_gain * self.curve_direction, -20.0, 20.0)
            elif self.lane_curvature > 500000:
                base_gain = self.lane_curvature / 500000.0
                curvature_gain = np.clip(base_gain * self.curve_direction, -17.0, 17.0)
            elif self.lane_curvature > 300000:
                base_gain = self.lane_curvature / 300000.0
                curvature_gain = np.clip(base_gain * self.curve_direction, -14.0, 14.0)
            elif self.lane_curvature > 200000:
                base_gain = self.lane_curvature / 200000.0
                curvature_gain = np.clip(base_gain * self.curve_direction, -12.0, 12.0)
            elif self.lane_curvature > 150000:
                base_gain = self.lane_curvature / 150000.0
                curvature_gain = np.clip(base_gain * self.curve_direction, -10.0, 10.0)

            steering_angle += curvature_gain

            if self.lane_status == "LEFT":
                steering_angle += 5.0
            elif self.lane_status == "RIGHT":
                steering_angle -= 5.0

        final_steer = self.smooth_steering_relative(steering_angle)
        final_steer = np.clip(final_steer, -20, 20)

        throttle = self.speed_pid.update_speed_error(target_speed, ego_speed)

        motor_cmd = XycarMotor()
        motor_cmd.header = Header()
        motor_cmd.header.stamp = rospy.Time.now()
        motor_cmd.header.frame_id = "map"
        motor_cmd.angle = int(final_steer * 5)
        motor_cmd.speed = int(throttle)

        self.prev_steer = final_steer

        pixel_error = self.target_x - IMAGE_CENTER
        curve_direction_str = "LEFT" if self.curve_direction == -1 else "RIGHT" if self.curve_direction == 1 else "STRAIGHT"

        rospy.loginfo_throttle(1.0,
            f"Lane - Status: {self.lane_status} | Direction: {curve_direction_str} | "
            f"Pixel Error: {pixel_error:+4d}px | "
            f"PID: {pid_steer:+5.1f}° | Curve: {curvature_gain:+5.1f}° | "
            f"Final: {final_steer:+5.1f}° | Curvature: {self.lane_curvature:.0f}"
        )

        return motor_cmd

    def get_status_relative(self):
        """
        현재 상대좌표 기반 상태를 딕셔너리 형태로 반환한다.
        """
        pixel_error = self.target_x - IMAGE_CENTER
        lateral_distance_error = (pixel_error / (IMAGE_WIDTH / 4)) * 2.0
        curve_direction_str = "LEFT" if self.curve_direction == -1 else "RIGHT" if self.curve_direction == 1 else "STRAIGHT"

        return {
            'coordinate_system': 'relative',
            'pixel_error': pixel_error,
            'lateral_distance_error_m': lateral_distance_error,
            'lane_status': self.lane_status,
            'lane_curvature': self.lane_curvature,
            'curve_direction': curve_direction_str,
            'target_x_px': self.target_x,
            'image_center_px': IMAGE_CENTER
        }

    def reset_controller(self):
        """
        제어기의 내부 상태(이전 오차, 적분 등)를 초기화한다.
        - lateral_pid.previous_error, lateral_pid.integral, lateral_pid.error_history 초기화
        - speed_pid.previous_error, speed_pid.integral 초기화
        - prev_steer, steer_history 초기화
        """
        self.lateral_pid.previous_error = 0.0
        self.lateral_pid.integral = 0.0
        self.lateral_pid.error_history = []

        self.speed_pid.previous_v = 0.0
        self.speed_pid.integral = 0.0

        self.prev_steer = 0.0
        self.steer_history = []