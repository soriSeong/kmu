import rospy
from xycar_msgs.msg import XycarMotor
from std_msgs.msg import Header
from controller.frenet_controller import FrenetController
from controller.lane_driving import LaneDrivingController
from controller.cone_driving import ConeDrivingController
from controller.lane_switcher import LaneSwitcher

# target_x 급격한 변화 감지 임계값 (픽셀 단위) 높을 수록 감지 민감도 낮아짐
TARGETX_JUMP_THRESHOLD = 110

# 후진 속도 및 지속 시간 설정
STOP_DURATION = 0.1       # Spike 발생 시 먼저 얼만큼 정지할지
REVERSE_SPEED = -8     # 후진 속도
REVERSE_DURATION = 1.5  # 후진 지속 시간(초)

class Motion:
    def __init__(self, shared):
        self.shared = shared
        self.plan = shared.plan
        self.ego = shared.ego
        self.perception = shared.perception
        
        # 상대좌표 기준 컨트롤러들 초기화
        self.frenet_controller = FrenetController()
        self.lane_controller = LaneDrivingController(shared)
        self.cone_controller = ConeDrivingController(shared)
        self.lane_switcher = LaneSwitcher(shared)

        # Spike 감지를 위한 이전 target_x 저장 변수
        self.prev_target_x = None

        # 후진 모드 관리 변수: None이면 후진 중 아님    
        self.reverse_end_time = None

        # Spike 발생시 정지
        self.stop_end_time = None

        # 모터 제어 퍼블리셔
        self.actuator_pub = rospy.Publisher('/xycar_motor', XycarMotor, queue_size=10)
        
        rospy.loginfo("Motion controller initialized (Relative Coordinate System)")
    
    def create_motor_command(self, angle, speed):
        """모터 명령 생성 유틸리티"""
        motor_msg = XycarMotor()
        motor_msg.header = Header()
        motor_msg.header.stamp = rospy.Time.now()
        motor_msg.header.frame_id = "map"
        motor_msg.angle = int(angle)
        motor_msg.speed = int(speed)
        return motor_msg
    
    def target_control(self, speed):
        """기본 속도 제어 (직진)"""
        if self.ego:
            self.ego.target_speed = int(speed)
        
        motor_msg = self.create_motor_command(0, speed)  # 직진
        self.actuator_pub.publish(motor_msg)
     
    def stop(self):
        """정지"""
        rospy.loginfo("Motion: Stop")
        motor_msg = self.create_motor_command(0, 0)
        self.actuator_pub.publish(motor_msg)
     
    def go(self):
        """차선 기반 주행 + spike 감지 → 정지 → 후진 → 정상 복귀"""
        rospy.loginfo("Motion: Lane following")
        now = rospy.Time.now()

        # 콘 주행 직후 한 번만 실행할 조향 + 전진
        if hasattr(self.shared, 'cone_exit_done') and self.shared.cone_exit_done:
            rospy.loginfo("Cone exit motion: right turn + short forward")
            transition_cmd = self.create_motor_command(angle=40, speed=7)
            for _ in range(30):  # 약 3초간 (0.1s * 30)
                self.actuator_pub.publish(transition_cmd)
                rospy.sleep(0.1)
            rospy.loginfo("Cone exit motion complete")
            self.shared.cone_exit_done = False
            return

        # STEP1: 정지 모드 처리: stop_end_time이 None이 아니면 먼저 정지만 수행
        if self.stop_end_time is not None:
            if now < self.stop_end_time:
                # 아직 정지 유지 시간(0.3초) 이내 → 계속 정지(속도=0)
                rospy.logwarn_throttle(1.0, "Stopping (preparing to reverse) ...")
                stop_cmd = self.create_motor_command(angle=0, speed=0)
                self.actuator_pub.publish(stop_cmd)
                return
            else:
                # 정지 시간이 끝난 시점 → 정지 모드 해제, 후진 모드 진입
                rospy.loginfo("Stop complete. Entering reverse mode.")
                self.stop_end_time = None
                self.reverse_end_time = now + rospy.Duration(REVERSE_DURATION)
                rev_cmd = self.create_motor_command(angle=0, speed=REVERSE_SPEED)
                self.actuator_pub.publish(rev_cmd)
                return

        # STEP2: 후진 모드 처리: reverse_end_time이 None이 아니면 계속 후진
        if self.reverse_end_time is not None:
            if now < self.reverse_end_time:
                # 아직 후진 유지 시간이내 계속 후진
                rospy.logwarn_throttle(1.0, "Reversing (due to target_x spike) ...")
                rev_cmd = self.create_motor_command(angle=0, speed=REVERSE_SPEED)
                self.actuator_pub.publish(rev_cmd)
                return
            else:
                # 후진 시간이 끝난 시점 → 후진 모드 해제, prev_target_x 초기화
                rospy.loginfo("Reversing complete. Resuming normal lane follow.")
                self.reverse_end_time = None
                self.prev_target_x = None
                # 마지막으로 한 번 더 후진 명령을 보내고 return
                rev_cmd = self.create_motor_command(angle=0, speed=REVERSE_SPEED)
                self.actuator_pub.publish(rev_cmd)
                return

        # STEP3: Spike 감지 (target_x 급격한 변화만 사용)
        curr_target_x = self.lane_controller.target_x
        jump_detected = False

        if self.prev_target_x is not None:
            if abs(curr_target_x - self.prev_target_x) > TARGETX_JUMP_THRESHOLD:
                jump_detected = True

        if jump_detected:
            # Spike 발생 시, 먼저 정지 모드 진입 (0.3초 동안)
            self.stop_end_time = now + rospy.Duration(STOP_DURATION)
            stop_cmd = self.create_motor_command(angle=0, speed=0)
            self.actuator_pub.publish(stop_cmd)
            return

        # Spike 미감지 시, 다음 호출을 위해 이전값 업데이트
        self.prev_target_x = curr_target_x

        rospy.loginfo_throttle(1.0, "Motion: Lane following")
        try:
            motor_cmd = self.lane_controller.lane_following_control()
            self.actuator_pub.publish(motor_cmd)
            rospy.sleep(0.1)
        except Exception as e:
            rospy.logerr(f"Lane following error: {e}")
            self.emergency_stop()

    def traffic_light(self):
        """신호등 제어"""
        # perception에서 신호등 정보 가져오기
        traffic_light_state = self.perception.traffic_light
        
        if traffic_light_state == False:
            rospy.loginfo("Traffic light: RED")
            self.stop()
        elif traffic_light_state == True:
            rospy.loginfo("Traffic light: GREEN")
            self.target_control(35)
        else:
            self.target_control(35)
     
    def obs_small(self):
        """작은 장애물 회피 - 라바콘 (상대좌표 기준으로 추종)"""
        rospy.loginfo("Motion: Small obstacle avoidance)")
        try:
            motor_cmd = self.cone_controller.follow_cone_path()

            if motor_cmd:
                self.actuator_pub.publish(motor_cmd)
                if not self.shared.cone_exit_done :
                    self.shared.cone_exit_done  = True
            else:
                rospy.loginfo("Cone driving finished, preparing to transition to go")
                
                self.plan.motion_decision = "go"

        except Exception as e:
            rospy.logerr(f"Cone driving error: {e}")
            self.target_control(10)  # 더 감속


    def obs_big(self):
        """차량 회피: LaneSwitcher 로 전환"""
        rospy.loginfo("Motion: Large obstacle avoidance via LaneSwitcher")

        left_has_obs  = len(self.shared.perception.left_obstacles)  > 0
        right_has_obs = len(self.shared.perception.right_obstacles) > 0

        if self.lane_switcher.update(left_has_obs, right_has_obs):
            return
        
        # LaneSwitcher가 False 를 반환했다는 것은
        # “지금은 장애물이 없거나, 회피 모드를 모두 마친 상태”이므로
        # 안전을 위해 일단 정지 (혹은 다른 기본 동작을 넣어도 됩니다)
        self._stop()

    def emergency_stop(self):
        """비상 정지"""
        rospy.logwarn("Emergency stop activated!")
        
        # 비상 정지 메시지 (브레이크)
        emergency_msg = self.create_motor_command(0, -10)
        
        # 여러 번 발행하여 확실히 정지
        for _ in range(5):
            self.actuator_pub.publish(emergency_msg)
            rospy.sleep(0.1)
        
        # 완전 정지
        final_stop = self.create_motor_command(0, 0)
        self.actuator_pub.publish(final_stop)

    def get_current_state(self):
        """현재 상태 반환 (상대좌표 기준)"""
        if self.ego:
            return {
                'coordinate_system': 'relative',
                'position': (self.ego.x, self.ego.y),  # 절대좌표 (참조용)
                'yaw': self.ego.yaw,
                'speed': self.ego.speed,
                'steer': self.ego.steer,
                'controllers': {
                    'lane_status': self.lane_controller.get_status_relative(),
                    'cone_status': self.cone_controller.get_status(),
                    'frenet_active': hasattr(self.shared, 'local_path') and self.shared.local_path is not None
                }
            }
        else:
            return {
                'coordinate_system': 'relative',
                'error': 'No ego vehicle state available'
            }

    def get_active_controller(self):
        if self.plan.motion_decision == "go":
            return "lane_controller"
        elif self.plan.motion_decision == "obs_small":
            return "cone_controller"
        elif self.plan.motion_decision == "obs_big":
            return "frenet_controller"
        elif self.plan.motion_decision == "traffic_light":
            return "traffic_light_controller"
        elif self.plan.motion_decision == "stop":
            return "stop_controller"
        else:
            return "unknown"