#!/usr/bin/env python3
import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from reeds_shepp_path import calc_optimal_path, STEP_SIZE

class RVizPurePursuitTest:
    def __init__(self):
        rospy.init_node('rviz_pure_pursuit_test')
        
        # Publishers
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)
        self.car_pub = rospy.Publisher('/current_car', PoseStamped, queue_size=1)
        self.goal_pub = rospy.Publisher('/goal_car', PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/pursuit_markers', MarkerArray, queue_size=1)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # Pure Pursuit 파라미터 (실제 코드와 동일)
        self.wheelbase = 0.25
        self.lookahead_distance = 0.3
        self.lookahead_gain = 0.1
        self.min_lookahead = 0.2
        self.max_lookahead = 0.6
        self.base_speed = 8
        self.goal_tolerance = 0.05
        
        # 시뮬레이션 상태
        self.path_points = []
        self.current_path_index = 0
        self.virtual_position = [0, 0, 0]
        self.goal_position = None
        self.lookahead_point = None
        self.steering_angle = 0
        
        # 타이머
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_callback)  # 10Hz
        
        rospy.loginfo("RViz Pure Pursuit 테스트 시작!")
        rospy.loginfo("RViz에서 '2D Nav Goal'로 목표점을 설정하세요.")
    
    def goal_callback(self, msg):
        """RViz에서 목표점 설정 시 호출"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        rospy.loginfo(f"새 목표점: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°")
        self.generate_test_path(x, y, yaw)
    
    def generate_test_path(self, goal_x, goal_y, goal_yaw):
        """테스트용 경로 생성"""
        start = (0.0, 0.0, 0.0)
        goal = (goal_x, goal_y, goal_yaw)
        
        optimal_path = calc_optimal_path(*start, *goal, 1.0, STEP_SIZE)
        
        if optimal_path is None:
            rospy.logwarn("경로 생성 실패!")
            return
            
        self.path_points = list(zip(optimal_path.x, optimal_path.y, optimal_path.yaw))
        self.current_path_index = 0
        self.goal_position = goal
        
        rospy.loginfo(f"경로 생성 완료: {len(self.path_points)}개 포인트")
        self.publish_path()
    
    def find_lookahead_point(self):
        """Pure Pursuit 코드와 동일한 로직"""
        if not self.path_points:
            return None, -1
            
        current_speed = abs(self.base_speed) * 0.1
        dynamic_lookahead = self.lookahead_gain * current_speed + self.lookahead_distance
        dynamic_lookahead = max(self.min_lookahead, min(dynamic_lookahead, self.max_lookahead))
        
        if self.current_path_index >= len(self.path_points):
            self.current_path_index = len(self.path_points) - 1
            
        virtual_current = self.path_points[self.current_path_index]
        self.virtual_position = list(virtual_current)
        
        for i in range(self.current_path_index, len(self.path_points)):
            point = self.path_points[i]
            distance = math.sqrt(
                (point[0] - virtual_current[0])**2 + 
                (point[1] - virtual_current[1])**2
            )
            
            if distance >= dynamic_lookahead:
                return point, i
        
        return self.path_points[-1], len(self.path_points) - 1
    
    def calculate_pure_pursuit_steering(self, lookahead_point):
        """Pure Pursuit 조향각 계산"""
        virtual_current = self.virtual_position
        
        dx = lookahead_point[0] - virtual_current[0]
        dy = lookahead_point[1] - virtual_current[1]
        
        cos_yaw = math.cos(virtual_current[2])
        sin_yaw = math.sin(virtual_current[2])
        
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        
        lookahead_dist = math.sqrt(local_x**2 + local_y**2)
        
        if lookahead_dist < 0.01:
            return 0.0
            
        alpha = math.atan2(local_y, local_x)
        steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), lookahead_dist)
        
        return math.degrees(steering_angle)
    
    def update_callback(self, event):
        """시뮬레이션 업데이트 콜백"""
        if not self.path_points:
            return
            
        # Lookahead 점 찾기
        self.lookahead_point, _ = self.find_lookahead_point()
        
        if self.lookahead_point:
            # 조향각 계산
            self.steering_angle = self.calculate_pure_pursuit_steering(self.lookahead_point)
        
        # 경로 진행도 업데이트
        if self.current_path_index < len(self.path_points) - 1:
            speed_factor = abs(self.base_speed) / 100.0
            progress_increment = max(1, int(speed_factor * 3))
            self.current_path_index = min(
                len(self.path_points) - 1,
                self.current_path_index + progress_increment
            )
        
        # 목표 도달 확인
        if self.is_goal_reached():
            rospy.loginfo("목표 지점 도달!")
        
        # RViz에 발행
        self.publish_all()
    
    def is_goal_reached(self):
        """목표 도달 확인"""
        if not self.path_points or not self.goal_position:
            return False
            
        goal_point = self.path_points[-1]
        virtual_current = self.virtual_position
        
        distance_to_goal = math.sqrt(
            (goal_point[0] - virtual_current[0])**2 + 
            (goal_point[1] - virtual_current[1])**2
        )
        
        return distance_to_goal < self.goal_tolerance or self.current_path_index >= len(self.path_points) - 1
    
    def publish_path(self):
        """경로 발행"""
        if not self.path_points:
            return
            
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        
        for point in self.path_points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            
            # 방향 설정
            quat = tf.transformations.quaternion_from_euler(0, 0, point[2])
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def publish_car_poses(self):
        """현재 차량과 목표 차량 위치 발행"""
        header = Header()
        header.frame_id = "map"
        header.stamp = rospy.Time.now()
        
        # 현재 차량 (가상 위치)
        if self.virtual_position:
            current_pose = PoseStamped()
            current_pose.header = header
            current_pose.pose.position.x = self.virtual_position[0]
            current_pose.pose.position.y = self.virtual_position[1]
            current_pose.pose.position.z = 0.0
            
            quat = tf.transformations.quaternion_from_euler(0, 0, self.virtual_position[2])
            current_pose.pose.orientation.x = quat[0]
            current_pose.pose.orientation.y = quat[1]
            current_pose.pose.orientation.z = quat[2]
            current_pose.pose.orientation.w = quat[3]
            
            self.car_pub.publish(current_pose)
        
        # 목표 차량
        if self.goal_position:
            goal_pose = PoseStamped()
            goal_pose.header = header
            goal_pose.pose.position.x = self.goal_position[0]
            goal_pose.pose.position.y = self.goal_position[1]
            goal_pose.pose.position.z = 0.0
            
            quat = tf.transformations.quaternion_from_euler(0, 0, self.goal_position[2])
            goal_pose.pose.orientation.x = quat[0]
            goal_pose.pose.orientation.y = quat[1]
            goal_pose.pose.orientation.z = quat[2]
            goal_pose.pose.orientation.w = quat[3]
            
            self.goal_pub.publish(goal_pose)
    
    def publish_markers(self):
        """마커 발행 (Lookahead 포인트 등)"""
        marker_array = MarkerArray()
        
        # Lookahead 포인트 마커
        if self.lookahead_point:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = self.lookahead_point[0]
            marker.pose.position.y = self.lookahead_point[1]
            marker.pose.position.z = 0.1
            
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
        
        # Lookahead 선 마커
        if self.lookahead_point and self.virtual_position:
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.id = 1
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            
            # 시작점 (현재 위치)
            start_point = Point()
            start_point.x = self.virtual_position[0]
            start_point.y = self.virtual_position[1]
            start_point.z = 0.0
            line_marker.points.append(start_point)
            
            # 끝점 (Lookahead 포인트)
            end_point = Point()
            end_point.x = self.lookahead_point[0]
            end_point.y = self.lookahead_point[1]
            end_point.z = 0.0
            line_marker.points.append(end_point)
            
            line_marker.scale.x = 0.05  # 선 두께
            
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 0.8
            
            marker_array.markers.append(line_marker)
        
        self.marker_pub.publish(marker_array)
    
    def publish_cmd_vel(self):
        """제어 명령 발행 (시각화용)"""
        cmd = Twist()
        
        # 속도 계산 (시각화용)
        if self.goal_position and self.virtual_position:
            distance_to_goal = math.sqrt(
                (self.goal_position[0] - self.virtual_position[0])**2 + 
                (self.goal_position[1] - self.virtual_position[1])**2
            )
            
            if distance_to_goal < 0.3:
                speed = 0.1
            elif distance_to_goal < 0.6:
                speed = 0.2
            else:
                speed = 0.3
            
            cmd.linear.x = speed
            cmd.angular.z = math.radians(self.steering_angle * 0.1)  # 조향각을 각속도로 변환
        
        self.cmd_pub.publish(cmd)
    
    def publish_all(self):
        """모든 정보 발행"""
        self.publish_car_poses()
        self.publish_markers()
        self.publish_cmd_vel()
        
        # 상태 정보 출력
        if self.path_points:
            progress = (self.current_path_index / len(self.path_points)) * 100
            rospy.loginfo_throttle(1.0, 
                f"진행률: {progress:.1f}%, 조향각: {self.steering_angle:.1f}°, "
                f"목표 도달: {'예' if self.is_goal_reached() else '아니오'}")

def main():
    try:
        test = RVizPurePursuitTest()
        
        # 초기 테스트 경로 생성
        rospy.sleep(1.0)  # 노드 초기화 대기
        test.generate_test_path(2.0, 1.5, math.pi/4)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("RViz 테스트 종료")

if __name__ == '__main__':
    main()