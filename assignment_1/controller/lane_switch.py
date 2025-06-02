#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from xycar_msgs.msg import XycarMotor
from math import cos, sin

######################### 파라 미터 튜닝 #########################
class LaneSwitcher:
    """
    맵 정보 없이, LiDAR(Perception.vehicle_list)에서 넘어오는 차량 리스트만으로
    하드코딩 방식의 차선 변경을 수행하는 클래스.
    Motion.planner에서 주기적으로 update()를 호출하면,
    내부 상태에 따라 정지 및 차선 변경 명령을 퍼블리시하거나,
    False를 반환해 다른 컨트롤러(예: LaneDrivingController)에게 제어를 위임합니다.
    """

    def __init__(self, shared):
        # ===== 하드코딩 파라미터 =====
        self.STOP_DURATION = rospy.get_param("~stop_duration", 3.0)   # 장애물 감지 시 정지 시간 (초)
        self.STEER_ANGLE   = rospy.get_param("~steer_angle", 20.0)    # 차선 전환 시 고정 스티어 각도 (deg)
        self.SWITCH_SPEED  = rospy.get_param("~switch_speed", 3.0)    # 차선 전환 시 속도 (m/s)
        self.TURN_DURATION = rospy.get_param("~turn_duration", 1.0)   # 차선 전환에 걸리는 시간 (초)
        self.CRUISE_SPEED  = rospy.get_param("~cruise_speed", 5.5)   # 직진 시 속도 (m/s)
        self.LANE_HALF_WIDTH = rospy.get_param("~lane_half_width", 0.5)  # 차선 반폭 (m)

        # ===== 공유 객체 =====
        self.shared = shared
        self.ego = shared.ego
        self.perception = shared.perception

        # ===== 내부 상태 =====
        self.curr_lane     = 1     # 1=왼쪽 차선, 2=오른쪽 차선
        self.is_stopping   = False
        self.is_switching = False
        self.stop_end_time = None
        self.turn_end_time = None

        # ===== 퍼블리셔 =====
        # LaneSwitcher가 자체적으로 '/xycar_motor' 명령 발행
        self.cmd_pub = rospy.Publisher("/xycar_motor", XycarMotor, queue_size=1)

        rospy.loginfo("LaneSwitcher initialized")

    def create_motor_cmd(self, angle_deg, speed):
        """XycarMotor 메시지 생성 헬퍼"""
        msg = XycarMotor()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.angle = int(angle_deg * 5)  # 1deg → 5 단위 매핑
        msg.speed = speed
        return msg

    def _begin_stop(self):
        """정지 모드 진입: STOP_DURATION 동안 속도=0, steer=0 명령을 계속 발행"""
        self.is_stopping = True
        self.stop_end_time = rospy.Time.now() + rospy.Duration(self.STOP_DURATION)

    def _process_stopping(self):
        """정지 모드 유지 중이면 속도=0명령 발행, 끝나면 차선 전환 모드로 전환"""
        now = rospy.Time.now()
        if now < self.stop_end_time:
            cmd = self.create_motor_cmd(angle_deg=0.0, speed=0.0)
            self.cmd_pub.publish(cmd)
            return True  # 아직 정지 중
        else:
            # 정지 시간 종료 → 차선 전환 모드 진입 준비
            self.is_stopping = False
            self.stop_end_time = None
            self.is_switching = True
            self.turn_end_time = rospy.Time.now() + rospy.Duration(self.TURN_DURATION)
            return False  # 정지 모드 종료

    def _process_switching(self, target_lane):
        """
        차선 전환 모드:
        TURN_DURATION 동안 STEER_ANGLE 고정 후,
        차선 변경 완료 시 angle=0, speed=CRUISE_SPEED 명령 발행
        """
        now = rospy.Time.now()
        if now < self.turn_end_time:
            # TURN_DURATION 동안 고정 스티어링
            if target_lane == 1:
                cmd = self.create_motor_cmd(angle_deg=+self.STEER_ANGLE, speed=self.SWITCH_SPEED)
            else:
                cmd = self.create_motor_cmd(angle_deg=-self.STEER_ANGLE, speed=self.SWITCH_SPEED)
            self.cmd_pub.publish(cmd)
            return True  # 아직 차선 전환 중
        else:
            # TURN_DURATION 지남 → 직진 모드로 복귀
            cmd_zero = self.create_motor_cmd(angle_deg=0.0, speed=self.CRUISE_SPEED)
            self.cmd_pub.publish(cmd_zero)
            self.is_switching = False
            self.turn_end_time = None
            return False  # 전환 모드 종료

    def _classify_vehicle_lanes(self):
        """
        Perception.vehicle_list(MarkerArray)에서 넘어온 차량 마커들을
        ego 기준으로 상대좌표 변환 후, left_has_obs, right_has_obs 반환

        - 전제: ego.yaw는 라디안 단위, ego.x/y는 절대 좌표
        - LAT/LON 변환: rel_x = cos(-yaw)*dx - sin(-yaw)*dy
                       rel_y = sin(-yaw)*dx + cos(-yaw)*dy
          여기서 rel_y > 0 → 차량 기준 왼쪽, rel_y < 0 → 오른쪽
        - 차선 폭은 LANE_HALF_WIDTH 사용
        """
        left_has_obs = False
        right_has_obs = False

        yaw = self.ego.yaw
        ex = self.ego.x
        ey = self.ego.y

        for marker in self.perception.vehicle_list:
            mx = marker.pose.position.x
            my = marker.pose.position.y
            dx = mx - ex
            dy = my - ey
            # ego 기준 상대 좌표
            rel_x =  cos(-yaw)*dx - sin(-yaw)*dy
            rel_y =  sin(-yaw)*dx + cos(-yaw)*dy

            # rel_x > 0 (전방)만 고려
            if rel_x <= 0:
                continue

            # rel_y > LANE_HALF_WIDTH → 왼쪽 차선, rel_y < -LANE_HALF_WIDTH → 오른쪽 차선
            if rel_y > self.LANE_HALF_WIDTH:
                left_has_obs = True
            elif rel_y < -self.LANE_HALF_WIDTH:
                right_has_obs = True

        return left_has_obs, right_has_obs

    def update(self):
        """
        주기적으로 호출:
          1) is_stopping/is_switching 상태가 True면 해당 모드로 제어 명령 발행
          2) Perception.vehicle_list에서 장애물 마커 분류 → left_has_obs, right_has_obs
          3) curr_lane에 따라 장애물이 있으면 stop→switch 모드 진입
          4) 나머지 경우 False 반환 (직진 제어는 상위 모듈에 위임)

        :return: True if LaneSwitcher가 명령을 퍼블리시했고,
                 False if 제어를 상위 모듈에 위임해야 함
        """
        # 1) 정지 모드 처리
        if self.is_stopping:
            still_stopping = self._process_stopping()
            # 정지 중이거나 차선 전환 모드로 넘어갔으면 True
            return True if still_stopping or self.is_switching else False

        # 2) 차선 전환 모드 처리
        if self.is_switching:
            still_switching = self._process_switching(self.curr_lane)
            return True if still_switching else False

        # 3) 차량 마커 분류 → left_has_obs, right_has_obs
        left_has_obs, right_has_obs = self._classify_vehicle_lanes()

        #    - curr_lane==1(왼쪽 차선 주행 중)이고 left_has_obs → 오른쪽 차선으로 회피
        if self.curr_lane == 1 and left_has_obs:
            rospy.loginfo("LaneSwitcher: Obstacle in left lane → stop then switch to right")
            self._begin_stop()
            # 차선 전환 목표를 반대 차선(2)로 설정
            self.curr_lane = 2
            return True

        #    - curr_lane==2(오른쪽 차선 주행 중)이고 right_has_obs → 왼쪽 차선으로 회피
        if self.curr_lane == 2 and right_has_obs:
            rospy.loginfo("LaneSwitcher: Obstacle in right lane → stop then switch to left")
            self._begin_stop()
            self.curr_lane = 1
            return True

        # 4) 장애물이 없거나 회피 모드 진입 전 → 제어 위임
        return False
