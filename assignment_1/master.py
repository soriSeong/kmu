import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
import threading
from shared import Shared
from mission_planner import MissionPlanner
from behavior_planner import BehaviorPlanner  
from motion_planner import MotionPlanner
import tf

class VehicleStateUpdater:
    def __init__(self, shared):
        self.shared = shared
        
        # 차량 상태 구독자
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_callback)
        
    def pose_callback(self, msg):
        self.shared.ego.x = msg.pose.position.x
        self.shared.ego.y = msg.pose.position.y
        # quaternion to euler 변환
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
    
    # 차량 상태 업데이터
    state_updater = VehicleStateUpdater(shared)
    
    # 각 플래너 생성 및 시작
    mission_planner = MissionPlanner(shared, rate=10)
    behavior_planner = BehaviorPlanner(shared, rate=20)
    motion_planner = MotionPlanner(shared, rate=20)
    
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