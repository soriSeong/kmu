#!/usr/bin/env python3
import rospy
from xycar_msgs.msg import XycarMotor
from std_msgs.msg import Header
from controller.frenet_controller import FrenetController
from controller.lane_driving import LaneDrivingController
from controller.cone_driving import ConeDrivingController
from controller.lane_switcher import LaneSwitcher

# target_x 급격한 변화 감지 임계값 (픽셀 단위) 높을수록 감지 민감도 낮아짐
TARGETX_JUMP_THRESHOLD = 110

# 후진 속도 및 지속 시간 설정
STOP_DURATION = 0.1       # Spike 발생 시 먼저 정지할 시간(초)
REVERSE_SPEED = -8        # 후진 속도
REVERSE_DURATION = 1.5    # 후진 지속 시간(초)

class Motion:
    def __init__(self, shared):
        """
        생성자
        - shared: Shared 객체에서 ego, perception, plan 객체를 가져옴
        - lane_controller: 차선 주행용 컨트롤러 인스턴스
        - cone_controller: 라바콘 회피용 컨트롤러 인스턴스
        
        - prev_target_x: Spike 감지를 위한 이전 프레임의 target_x 저장 

        - reverse_end_time: 후진 모드 종료 시간 (None이면 후진 중 아님)
        - stop_end_time: Spike 발생 시 정지 모드 종료 시간 (None이면 정지 중 아님)
        - actuator_pub: '/xycar_motor' 토픽 퍼블리셔 (XycarMotor 메시지 발행한다.)
        """
        self.shared = shared
        self.plan = shared.plan
        self.ego = shared.ego
        self.perception = shared.perception
        
        # 컨트롤러 초기화
        self.lane_controller = LaneDrivingController(shared)
        self.cone_controller = ConeDrivingController(shared)
        self.lane_switcher = LaneSwitcher(shared)

        self.prev_target_x = None
        self.reverse_end_time = None
        self.stop_end_time = None
        self.actuator_pub = rospy.Publisher('/xycar_motor', XycarMotor, queue_size=10)
        
        rospy.loginfo("Motion controller initialized (Relative Coordinate System)")

    def create_motor_command(self, angle, speed):
        """
        모터 제어 메시지 생성
        1. XycarMotor 메시지 객체 생성
        2. Header 설정 (ros 시간, frame_id="map")
        3. angle, speed 설정 후 반환
        """
        motor_msg = XycarMotor()
        motor_msg.header = Header()
        motor_msg.header.stamp = rospy.Time.now()
        motor_msg.header.frame_id = "map"
        motor_msg.angle = int(angle)
        motor_msg.speed = int(speed)
        return motor_msg
    
    def target_control(self, speed):
        """
        기본 속도 제어(직진)
        1. ego.target_speed 갱신 (ego 객체는 Shared에서 가져옴)
        2. 조향 각도 0으로 모터 메시지 생성 및 발행

        신호등 초록불일 때 사용
        """
        if self.ego:
            self.ego.target_speed = int(speed)
        
        motor_msg = self.create_motor_command(0, speed)
        self.actuator_pub.publish(motor_msg)
     
    def stop(self):
        """
        정지 모드
        1. 로그 출력("Motion: Stop")
        2. 조향 0, 속도 0인 모터 메시지 생성 및 발행
        """
        rospy.loginfo("Motion: Stop")
        motor_msg = self.create_motor_command(0, 0)
        self.actuator_pub.publish(motor_msg)
     
    def go(self):
        """
        기본적인 차선 인식 기반 주행, 
        Spike 감지 시 -> 정지 -> 후진 -> 정상 복귀

        - cone_exit_done 플래그가 True이면 라바콘 회피 종료 후 차선 복귀 동작(우회전 + 전진) 수행

        --- spike 발생 시 로직 ---
        - stop_end_time이 설정되어 있으면 정지 모드 유지 (속도=0)
        - reverse_end_time이 설정되어 있으면 후진 모드 유지 (속도=REVERSE_SPEED)
        - Spike 감지: lane_controller.target_x와 prev_target_x 차이 > 임계값이면 stop_end_time 설정 후 정지
        - Spike 미감지 시 lane_following_control 호출하여 차선 주행 명령 생성 및 발행
        """
        rospy.loginfo("Motion: Lane following")
        now = rospy.Time.now()

        # 라바콘 회피 종료 후 차선 복귀 동작
        if hasattr(self.shared, 'cone_exit_done') and self.shared.cone_exit_done:
            transition_cmd = self.create_motor_command(angle=40, speed=7)
            for _ in range(30):  # 약 3초간 (0.1s * 30)
                self.actuator_pub.publish(transition_cmd)
                rospy.sleep(0.1)
            rospy.loginfo("Cone exit motion complete")
            self.shared.cone_exit_done = False
            return

        if self.stop_end_time is not None:
            if now < self.stop_end_time:
                stop_cmd = self.create_motor_command(angle=0, speed=0)
                self.actuator_pub.publish(stop_cmd)
                return
            else:
                self.stop_end_time = None
                self.reverse_end_time = now + rospy.Duration(REVERSE_DURATION)
                rev_cmd = self.create_motor_command(angle=0, speed=REVERSE_SPEED)
                self.actuator_pub.publish(rev_cmd)
                return

        if self.reverse_end_time is not None:
            if now < self.reverse_end_time:
                rev_cmd = self.create_motor_command(angle=0, speed=REVERSE_SPEED)
                self.actuator_pub.publish(rev_cmd)
                return
            else:
                self.reverse_end_time = None
                self.prev_target_x = None
                rev_cmd = self.create_motor_command(angle=0, speed=REVERSE_SPEED)
                self.actuator_pub.publish(rev_cmd)
                return

        curr_target_x = self.lane_controller.target_x  # LaneDrivingController에서 유지되는 값
        jump_detected = False

        if self.prev_target_x is not None:
            if abs(curr_target_x - self.prev_target_x) > TARGETX_JUMP_THRESHOLD:
                jump_detected = True

        if jump_detected:
            delta = abs(curr_target_x - self.prev_target_x)
            rospy.logwarn(f"Spike detected! Δtarget_x={delta} > {TARGETX_JUMP_THRESHOLD}")
            # Spike 발생 시 정지 모드 진입 (STOP_DURATION 동안)
            self.stop_end_time = now + rospy.Duration(STOP_DURATION)
            stop_cmd = self.create_motor_command(angle=0, speed=0)
            self.actuator_pub.publish(stop_cmd)
            return

        # Spike 미감지 시, 다음 호출을 위해 이전 target_x 값 업데이트
        self.prev_target_x = curr_target_x

        rospy.loginfo_throttle(1.0, "Motion: Lane following")
        try:
            motor_cmd = self.lane_controller.lane_following_control()
            self.actuator_pub.publish(motor_cmd)
            rospy.sleep(0.1)
        except Exception as e:
            rospy.logerr(f"Lane following error: {e}")

    def traffic_light_decision(self):
        """
        신호등 제어
        perception.traffic_light 속성(True=초록불, False=빨간불 or 노란불) 참조
        빨간불 or 노란불(False) -> stop() 호출
        초록불(True) → target_control(35) 호출 (직진 속도 35)
        """
        traffic_light_state = self.perception.traffic_light
        
        if traffic_light_state is False:  # 빨간불 또는 노란불
            rospy.loginfo("Traffic light: RED")
            self.stop()
        else:
            rospy.loginfo("Traffic light: GREEN")
            self.target_control(35)

    def obs_small(self):
        """
        작은 장애물 회피 - 라바콘 (상대좌표 기준) 라이다에서 넘겨주는 middle path를 활용해 라바콘을 회피한다.
        cone_controller.follow_cone_path() 호출해 조향/속도 명령 반환 여부 확인
        명령이 있으면 발행, 향후 한 번만 실행할 전환 플래그(cone_exit_done) 설정
        명령이 없으면 라바콘 회피 완료로 간주, plan.motion_decision = "go" 설정
        """
        rospy.loginfo("Motion: Small obstacle avoidance")
        try:
            motor_cmd = self.cone_controller.follow_cone_path()

            if motor_cmd:
                self.actuator_pub.publish(motor_cmd)
                if not getattr(self.shared, 'cone_exit_done', False):
                    self.shared.cone_exit_done = True
            else:
                rospy.loginfo("Cone driving finished, preparing to transition to go")
                self.plan.motion_decision = "go"

        except Exception as e:
            rospy.logerr(f"Cone driving error: {e}")
            # 오류 시 속도를 10으로 감속하여 직진
            self.target_control(10)

    def obs_big(self):
        """
        LaneSwitcher를 사용한 차량 회피
        """
        rospy.loginfo("Vehicle avoidance using LaneSwitcher")

        # 좌/우 차선 장애물 리스트를 Boolean 으로 판별
        left_has_obs  = len(self.shared.perception.left_obstacles)  > 0
        right_has_obs = len(self.shared.perception.right_obstacles) > 0

        # LaneSwitcher 가 True 반환 시, 내부에서 정지 또는 차선 전환 명령을 퍼블리시 했으므로 제어 위임 끝
        if self.lane_switcher.update(left_has_obs, right_has_obs):
            return

        # 회피 모드가 아닌 경우에는 일단 정지(또는 다른 기본 동작)
        self.stop()