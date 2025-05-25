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
    """parent 객체"""
    def __init__(self, shared):
        self.plan = shared.plan    
        self.shared = shared

class VehicleStateUpdater:
    def __init__(self, shared):
        self.shared = shared
        
        # 차량 상태 Subscriber
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_callback)
        
    def pose_callback(self, msg):
        self.shared.ego.x = msg.pose.position.x
        self.shared.ego.y = msg.pose.position.y
        orientation = msg.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.shared.ego.yaw = euler[2]
        
    def velocity_callback(self, msg):
        self.shared.ego.speed = msg.twist.linear.x

def main():
    rospy.init_node('autonomous_driving_master')
    
    # Shared 객체 생성
    shared = Shared()
    
    # Parent 객체 생성 (기존 planner들을 위해)
    parent = Parent(shared)
    
    # 차량 상태 업데이터
    state_updater = VehicleStateUpdater(shared)
    
    # planner 생성 (parent 객체 전달)
    mission_planner = MissionPlanner(parent, rate=10)
    behavior_planner = BehaviorPlanner(parent, rate=20)
    motion_planner = MotionPlanner(parent, rate=20)
    
    # daemon 설정 컨트롤 + c로 종료
    mission_planner.daemon = True
    behavior_planner.daemon = True
    motion_planner.daemon = True
    
    # 스레드 시작
    mission_planner.start()
    behavior_planner.start()
    motion_planner.start()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down autonomous driving system")

if __name__ == '__main__':
    main()