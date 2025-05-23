import math
import rospy
from geometry_msgs.msg import PoseStamped
from xycar_msgs.msg import XycarMotor
from planning.reeds_shepp_path import calc_optimal_path, pi_2_pi, STEP_SIZE
from control.pure_pursuit import pure_pursuit, pid_control, PATH, Node, Nodes, C
import tf

class Parking:
    def __init__(self):
        rospy.init_node("parking_node")
        self.motor_pub = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)
        rospy.Subscriber("/apriltag_pose", PoseStamped, self.callback_apriltag)

        self.goal = None
        self.start = (0.0, 0.0, 0.0)
        self.cur_path = None
        self.path_follow = None
        self.index_old = 0
        self.node = Node(*self.start, v=0.0, direct=1)
        self.nodes = Nodes()
        self.rate = rospy.Rate(10)

    def plan_path(self):
        sx, sy, syaw = self.start
        gx, gy, gyaw = self.goal
        max_curvature = 1.0
        self.cur_path = calc_optimal_path(sx, sy, syaw, gx, gy, gyaw, max_curvature, STEP_SIZE)

        if self.cur_path is None:
            rospy.logerr("경로 생성 실패")
            return False

        self.path_follow = PATH(self.cur_path.x, self.cur_path.y)
        return True

    def callback_apriltag(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation

        # EulerAngles = Quaternion -> yaw 변환
        quaternion = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = quaternion[2]

        self.goal = (x, y, yaw)

        if self.plan_path():
            self.drive()

    def drive(self):
        rospy.loginfo("주행 시작")

        while not rospy.is_shutdown():
            if self.path_follow is None:
                break

            delta, self.index_old = pure_pursuit(self.node, self.path_follow, self.index_old)
            dist = self.path_follow.calc_distance(self.node, self.index_old)
            a = pid_control(1.5, self.node.v, dist, 1)

            self.node.update(a, delta, 1)
            self.nodes.add(rospy.get_time(), self.node)

            msg = XycarMotor()
            msg.angle = delta
            msg.speed = a
            self.motor_pub.publish(msg)

            if self.index_old >= self.path_follow.ind_end:
                rospy.loginfo("목표 도달")
                break
            
            self.rate.sleep()
