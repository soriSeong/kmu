import rospy
from xycar_msgs.msg import XycarMotor
from std_msgs.msg import Header
from controller.frenet_controller import FrenetController
from controller.lane_driving import LaneDrivingController

class Motion():
    def __init__(self, shared):
        self.shared = shared
        self.plan = shared.plan
        self.ego = shared.ego
        self.perception = shared.perception
        
        # 컨트롤러 초기화
        self.frenet_controller = FrenetController()
        self.lane_controller = LaneDrivingController(shared)
        
        # 모터 제어 퍼블리셔
        self.actuator_pub = rospy.Publisher('/xycar_motor', XycarMotor, queue_size=10)
        
        rospy.loginfo("Motion controller initialized")
            
    def target_control(self, speed):
        """기본 속도 제어"""
        if self.ego:
            self.ego.target_speed = int(speed)
        
        # XycarMotor 메시지 생성 및 발행
        motor_msg = XycarMotor()
        motor_msg.header = Header()
        motor_msg.header.stamp = rospy.Time.now()
        motor_msg.header.frame_id = "map"
        motor_msg.angle = 0  # 직진
        motor_msg.speed = speed
        
        self.actuator_pub.publish(motor_msg)
     
    def stop(self):
        """정지"""
        rospy.loginfo("Motion: Stop")
        self.target_control(0)
     
    def go(self):
        """차선 주행 - LaneDrivingController 사용"""
        rospy.loginfo("Motion: Lane following")
        try:
            # 차선 주행 컨트롤러 사용
            motor_cmd = self.lane_controller.lane_following_control()
            
        except Exception as e:
            rospy.logerr(f"Lane following error: {e}")
            # 오류 시 비상 정지
            self.lane_controller.emergency_stop()
     
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
            rospy.logwarn("Traffic light: Unknown state")
            self.stop()
     
    def obs_small(self):
        """작은 장애물 회피 - 라바콘"""
        rospy.loginfo("Motion: Small obstacle avoidance")
        
        if hasattr(self.perception, 'middle_path') and self.perception.middle_path:
            try:
                # middle path를 따라 주행
                self.target_control(15)  # 감속 주행
                rospy.loginfo("Following middle path for obstacle avoidance")
                
            except Exception as e:
                rospy.logerr(f"Middle path following error: {e}")
                self.target_control(10)  # 더 감속
        else:
            # middle path가 없으면 에러메시지, 속도 감속속
            rospy.logwarn("No middle path available, using simple speed control")
            self.target_control(10)

    def obs_big(self):
        """차량 회피 - Optimal Frenet + 제어 실행"""
        rospy.loginfo("Motion: Big obstacle (vehicle) avoidance")
        
        # Frenet 경로가 생성되었는지 확인
        path = self.shared.local_path
        if not path or not hasattr(path, 'x') or len(path.x) < 2:
            rospy.logwarn("No valid Frenet path available")
            self.target_control(35)
            return

        try:
            # Frenet 컨트롤러로 경로 추종
            msgs = self.frenet_controller.follow_frenet_path(
                path,
                ego_x=self.ego.x if self.ego else 0,
                ego_y=self.ego.y if self.ego else 0,
                yaw=self.ego.yaw if self.ego else 0,
                speed=self.ego.speed if self.ego else 5,
                steer_prev=self.ego.steer if self.ego else 0
            )

            if msgs:
                # 생성된 제어 메시지들을 순차적으로 발행
                for msg in msgs:
                    self.actuator_pub.publish(msg)
                    rospy.sleep(0.05)  # 짧은 간격으로 발행
                    
                rospy.loginfo("Frenet path following commands sent")
            else:
                rospy.logwarn("No valid control messages from Frenet controller")
                self.target_control(0)  # 정지
                
        except Exception as e:
            rospy.logerr(f"Frenet path following error: {e}")
            self.target_control(0)  # 오류 시 정지

    def emergency_stop(self):
        """비상 정지"""
        rospy.logwarn("Emergency stop activated!")
        motor_msg = XycarMotor()
        motor_msg.header = Header()
        motor_msg.header.stamp = rospy.Time.now()
        motor_msg.header.frame_id = "map"
        motor_msg.angle = 0
        motor_msg.speed = -10  # 정지
        
        # 여러 번 발행하여 확실히 정지
        for _ in range(5):
            self.actuator_pub.publish(motor_msg)
            rospy.sleep(0.1)
        
        # 완전 정지
        motor_msg.speed = 0
        self.actuator_pub.publish(motor_msg)

    def get_current_state(self):
        """현재 상태 반환"""
        if self.ego:
            return {
                'position': (self.ego.x, self.ego.y),
                'yaw': self.ego.yaw,
                'speed': self.ego.speed,
                'steer': self.ego.steer
            }
        else:
            return None