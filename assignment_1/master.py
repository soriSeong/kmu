#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
import threading
import tf
from shared.shared import Shared
from planning.mission_planner import MissionPlanner
from planning.behavior_planner import BehaviorPlanner   
from planning.motion_planner import MotionPlanner
from controller.frenet_controller import FrenetController
from controller.lane_driving import LaneDrivingController

class Parent:
    """
    Parent 클래스:
    상위 노드의 Shared 객체를 래핑하여 Planner들에게 전달한다.
    shared: Shared 객체 참조 (모든 Planner, Controller가 공통으로 사용하는 전역 상태) 한다.
    plan: Shared.plan을 바로 참조하여 Mission/Behavior/Motion 모듈 간 동기화를 진행한다.
    """
    def __init__(self, shared):
        self.plan = shared.plan    
        self.shared = shared

class VehicleStateUpdater:
    """
    차량 상태 업데이트 클래스:
    ROS 토픽 '/current_pose', '/current_velocity'를 구독하여,
    Shared.ego 객체의 x, y, yaw, speed를 실시간으로 갱신한다.
    """
    def __init__(self, shared):
        """
        shared: Shared 객체, ego 상태를 저장할 공간

        shared.ego 참조 저장
        '/current_pose' 토픽 Subscriber: pose_callback 호출
        '/current_velocity' 토픽 Subscriber: velocity_callback 호출
        """
        self.shared = shared
        
        # 차량 위치/방향 구독자
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        # 차량 속도 구독자
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_callback)
        
    def pose_callback(self, msg):
        """
        '/current_pose' 콜백
        msg.pose.position: 차량 위치
        msg.pose.orientation: 차량 자세(quaternion) 이후 변환을 진행한다.
        
        self.shared.ego.x, self.shared.ego.y에 좌표 저장한다.
        quaternion을 tf 변환 후 self.shared.ego.yaw에 저장한다.
        """
        self.shared.ego.x = msg.pose.position.x
        self.shared.ego.y = msg.pose.position.y
        orientation = msg.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.shared.ego.yaw = euler[2]
        
    def velocity_callback(self, msg):
        """
        '/current_velocity' 콜백
        msg.twist.linear.x: 차량 속도
        self.shared.ego.speed에 현재 속도 저장한다.
        """
        self.shared.ego.speed = msg.twist.linear.x

def main():
    """
    ROS 노드 초기화('autonomous_driving_car')

    Shared 객체 생성: 전역 상태를 관리한다.

    VehicleStateUpdater 인스턴스 생성: ego 상태를 갱신한다.

    MissionPlanner, BehaviorPlanner, MotionPlanner 인스턴스 생성 (각각 rate=20Hz)한다.

    Planner 스레드를 데몬 설정(메인 종료 시 자동 종료한다).
    각 Planner 스레드를 시작한다.
    rospy.spin()으로 콜백 루프를 유지한다. Ctrl+C 시 안전 종료 로그를 출력한다.
    """
    rospy.init_node('autonomous_driving_car')
    
    shared = Shared()
    
    parent = Parent(shared)
    
    state_updater = VehicleStateUpdater(shared)
    
    mission_planner = MissionPlanner(parent, rate=20)
    behavior_planner = BehaviorPlanner(parent, rate=20)
    motion_planner = MotionPlanner(parent, rate=20)
    
    mission_planner.daemon = True
    behavior_planner.daemon = True
    motion_planner.daemon = True

    mission_planner.start()
    behavior_planner.start()
    motion_planner.start()
    
    try:
        # Ctrl+C 시 종료 처리
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down autonomous driving system")

if __name__ == '__main__':
    main()
