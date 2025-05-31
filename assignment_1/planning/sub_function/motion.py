import rospy
from xycar_msgs.msg import XycarMotor
from std_msgs.msg import Header
from controller.frenet_controller import FrenetController
from controller.lane_driving import LaneDrivingController
from controller.cone_driving import ConeDrivingController


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
        """차선 기반 주행"""
        rospy.loginfo("Motion: Lane following")

        # 콘 주행 직후 한 번만 실행할 조향 + 전진
        if hasattr(self.shared, 'cone_exit_done') and self.shared.cone_exit_done:
            rospy.loginfo("Cone exit motion: right turn + short forward")
            
            transition_cmd = self.create_motor_command(angle=40, speed=7)
            for _ in range(30):  # 2초간 (0.1s * 10)
                self.actuator_pub.publish(transition_cmd)
                rospy.sleep(0.1)

            rospy.loginfo("Cone exit motion complete")
            self.shared.cone_exit_done = False  # 다시 실행되지 않도록
            return  # 아래 일반 lane follow는 이번 루프에서는 생략

        # 이후 정상 lane follow
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
        
        if traffic_light_state == False:  # 빨간불 또는 황색불
            rospy.loginfo("Traffic light: RED")
            self.stop()
        elif traffic_light_state == True:  # 초록불
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
        """차량 회피 - Optimal Frenet (상대좌표 기준)"""
        rospy.loginfo("Motion: Large obstacle avoidance (Relative Coordinate)")
        
        # Frenet 경로가 생성되었는지 확인
        path = self.shared.local_path
        if not path or not hasattr(path, 'x') or len(path.x) < 2:
            rospy.logwarn("No valid Frenet path")
            self.target_control(35)
            return

        try:
            # 현재 차량 상태
            ego_x = self.ego.x if self.ego else 0
            ego_y = self.ego.y if self.ego else 0
            ego_yaw = self.ego.yaw if self.ego else 0
            ego_speed = self.ego.speed if self.ego else 5
            ego_steer = self.ego.steer if self.ego else 0
            
            # 상대좌표 기준 Frenet 경로 추종
            motor_commands = self.frenet_controller.follow_frenet_path(
                path,
                ego_x=ego_x,
                ego_y=ego_y,
                yaw=ego_yaw,
                speed=ego_speed,
                steer_prev=ego_steer
            )

            if motor_commands:
                # 생성된 제어 명령들을 순차적으로 발행
                for cmd in motor_commands:
                    self.actuator_pub.publish(cmd)
                    rospy.sleep(0.05)  # 짧은 간격으로 발행
                    
                rospy.loginfo("Frenet path following commands sent (Relative)")
            else:
                rospy.logwarn("No valid control messages from Frenet controller")
                self.stop()  # 정지
                
        except Exception as e:
            rospy.logerr(f"Frenet path following error: {e}")
            self.stop()  # 오류 시 정지

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

    def reset_all_controllers(self):
        """모든 제어기 상태 초기화"""
        rospy.loginfo("Resetting all controllers to relative coordinate system")
        
        # 각 컨트롤러 리셋
        if hasattr(self.lane_controller, 'reset_controller'):
            self.lane_controller.reset_controller()
        
        if hasattr(self.frenet_controller, 'reset'):
            self.frenet_controller.reset()
        
        rospy.loginfo("All controllers reset complete")

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