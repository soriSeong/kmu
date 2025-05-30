#!/usr/bin/env python3
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped
from xycar_msgs.msg import XycarMotor
from reeds_shepp_path import calc_optimal_path, STEP_SIZE

class RelativePurePursuit:
    def __init__(self):
        rospy.init_node("relative_pure_pursuit")
        
        # Publishers & Subscribers
        self.motor_pub = rospy.Publisher('/xycar_motor', XycarMotor, queue_size=1)
        rospy.Subscriber("/apriltag_pose", PoseStamped, self.apriltag_callback)
        
        # 차량 파라미터
        self.wheelbase = 0.25  # 25cm 휠베이스
        self.max_steer_angle = 30  # 최대 조향각 (degrees)
        
        # Pure Pursuit 파라미터
        self.lookahead_distance = 0.3  # 전방 주시 거리
        self.lookahead_gain = 0.1  # 속도에 따른 주시거리 조정 게인
        self.min_lookahead = 0.2   # 최소 주시거리
        self.max_lookahead = 0.6   # 최대 주시거리
        
        # 경로 추종 상태
        self.current_position = [0.0, 0.0, 0.0]  # 항상 원점 기준 (상대위치)
        self.path_points = []
        self.current_path_index = 0
        self.goal_tolerance = 0.1  # 목표 도달 허용 오차
        
        # 제어 패러미터 조정
        self.base_speed = 8
        self.control_rate = 20  # 20Hz
        self.goal_tolerance = 0.05  # 목표 도달 허용 오차를 더 엄격하게
        
    def apriltag_callback(self, msg):
        """AprilTag 검출 시 호출되는 콜백"""
        # AprilTag 위치 추출
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        goal_position = (x, y, yaw)
        rospy.loginfo(f"AprilTag 검출: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°")
        
        # 경로 생성 및 추종 시작
        self.plan_and_follow_path(goal_position)
        
    def plan_and_follow_path(self, goal):
        """경로 계획 및 추종 실행"""
        start = tuple(self.current_position)
        
        # Reeds-Shepp 경로 생성
        optimal_path = calc_optimal_path(*start, *goal, 1.0, STEP_SIZE)
        
        if optimal_path is None:
            rospy.logwarn("경로 생성 실패")
            return
            
        # 경로 포인트 저장
        self.path_points = list(zip(optimal_path.x, optimal_path.y, optimal_path.yaw))
        self.current_path_index = 0
        
        rospy.loginfo(f"경로 생성 완료: {len(self.path_points)}개 포인트")
        
        # 경로 추종 시작
        self.follow_path()
        
    def follow_path(self):
        """Pure Pursuit을 이용한 경로 추종"""
        rate = rospy.Rate(self.control_rate)
        
        while not rospy.is_shutdown() and self.current_path_index < len(self.path_points):
            # Lookahead 포인트 찾기
            lookahead_point, target_index = self.find_lookahead_point()
            
            if lookahead_point is None:
                rospy.logwarn("Lookahead 포인트를 찾을 수 없습니다")
                break
                
            # Pure Pursuit 조향각 계산
            steering_angle = self.calculate_pure_pursuit_steering(lookahead_point)
            
            # 속도 계산 (목표 근처에서 감속)
            speed = self.calculate_speed()
            
            # 모터 제어 명령 발행
            self.publish_motor_command(steering_angle, speed)
            
            # 경로 인덱스 업데이트 (상대위치에서는 시간 기반으로 진행)
            self.update_path_progress()
            
            # 목표 도달 확인
            if self.is_goal_reached():
                rospy.loginfo("목표 지점 도달!")
                break
                
            rate.sleep()
            
        # 정지
        self.publish_motor_command(0, 0)
        rospy.loginfo("경로 추종 완료")
        
    def find_lookahead_point(self):
        """경로 진행도를 고려한 lookahead 포인트 찾기"""
        if not self.path_points:
            return None, -1
            
        # 동적 lookahead 거리 계산
        current_speed = abs(self.base_speed) * 0.1
        dynamic_lookahead = self.lookahead_gain * current_speed + self.lookahead_distance
        dynamic_lookahead = max(self.min_lookahead, min(dynamic_lookahead, self.max_lookahead))
        
        # 현재 진행 지점을 가상의 현재 위치로 설정
        if self.current_path_index >= len(self.path_points):
            self.current_path_index = len(self.path_points) - 1
            
        virtual_current = self.path_points[self.current_path_index]
        
        # 가상 현재 위치에서 lookahead 거리만큼 떨어진 포인트 찾기
        for i in range(self.current_path_index, len(self.path_points)):
            point = self.path_points[i]
            distance = math.sqrt(
                (point[0] - virtual_current[0])**2 + 
                (point[1] - virtual_current[1])**2
            )
            
            if distance >= dynamic_lookahead:
                return point, i
        
        # 경로 끝에 도달한 경우 마지막 포인트 반환
        return self.path_points[-1], len(self.path_points) - 1
        
    def calculate_pure_pursuit_steering(self, lookahead_point):
        """Pure Pursuit 알고리즘을 이용한 조향각 계산 (가상 현재 위치 기준)"""
        # 가상의 현재 위치 (경로 진행도 기준)
        if self.current_path_index < len(self.path_points):
            virtual_current = self.path_points[self.current_path_index]
        else:
            virtual_current = self.path_points[-1]
        
        # 가상 현재 위치와 목표 포인트 사이의 상대 위치
        dx = lookahead_point[0] - virtual_current[0]
        dy = lookahead_point[1] - virtual_current[1]
        
        # 가상 현재 위치의 방향을 고려한 좌표 변환
        cos_yaw = math.cos(virtual_current[2])
        sin_yaw = math.sin(virtual_current[2])
        
        # 차량 좌표계에서의 목표점 위치
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        
        # Lookahead 거리
        lookahead_dist = math.sqrt(local_x**2 + local_y**2)
        
        if lookahead_dist < 0.01:  # 너무 가까우면 직진
            return 0.0
            
        # Pure Pursuit 공식: delta = atan2(2 * L * sin(alpha), Ld)
        alpha = math.atan2(local_y, local_x)
        steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), lookahead_dist)
        
        # 조향각을 도(degree)로 변환 및 제한
        steering_degrees = math.degrees(steering_angle)
        steering_degrees = max(-self.max_steer_angle, min(steering_degrees, self.max_steer_angle))
        
        return steering_degrees
        
    def calculate_speed(self):
        """현재 상황에 맞는 속도 계산"""
        if not self.path_points:
            return 0
            
        # 가상 현재 위치에서 목표점까지의 거리 계산
        goal_point = self.path_points[-1]
        if self.current_path_index < len(self.path_points):
            virtual_current = self.path_points[self.current_path_index]
        else:
            virtual_current = self.path_points[-1]
            
        distance_to_goal = math.sqrt(
            (goal_point[0] - virtual_current[0])**2 + 
            (goal_point[1] - virtual_current[1])**2
        )
        
        # 목표 근처에서 점진적 감속
        if distance_to_goal < 0.3:
            return max(3, int(self.base_speed * 0.4))  # 매우 느리게
        elif distance_to_goal < 0.6:
            return max(4, int(self.base_speed * 0.6))  # 느리게
        elif distance_to_goal < 1.0:
            return max(6, int(self.base_speed * 0.8))  # 조금 느리게
        else:
            return self.base_speed  # 정상 속도
            
    def update_path_progress(self):
        """경로 진행 상황 업데이트 (속도 기반)"""
        if self.current_path_index < len(self.path_points) - 1:
            # 현재 속도에 기반한 진행률 계산
            speed_factor = abs(self.base_speed) / 100.0  # 속도를 0-1 범위로 정규화
            progress_increment = max(1, int(speed_factor * 3))  # 최소 1, 최대 3씩 진행
            
            self.current_path_index = min(
                len(self.path_points) - 1,
                self.current_path_index + progress_increment
            )
            
            # 진행 상황 로그
            progress_percent = (self.current_path_index / len(self.path_points)) * 100
            rospy.logdebug(f"경로 진행률: {progress_percent:.1f}% ({self.current_path_index}/{len(self.path_points)})")
            
    def is_goal_reached(self):
        """목표 도달 여부 확인 (실제 목표점 기준)"""
        if not self.path_points:
            return True
            
        # 경로의 마지막 포인트 (실제 AprilTag 위치)
        goal_point = self.path_points[-1]
        
        # 가상 현재 위치에서 목표점까지의 거리
        if self.current_path_index < len(self.path_points):
            virtual_current = self.path_points[self.current_path_index]
        else:
            virtual_current = self.path_points[-1]
            
        distance_to_goal = math.sqrt(
            (goal_point[0] - virtual_current[0])**2 + 
            (goal_point[1] - virtual_current[1])**2
        )
        
        # 목표점에 충분히 가까이 도달했는지 확인
        return distance_to_goal < self.goal_tolerance or self.current_path_index >= len(self.path_points) - 1
        
    def publish_motor_command(self, steering_angle, speed):
        """모터 제어 명령 발행"""
        msg = XycarMotor()
        msg.angle = int(max(-self.max_steer_angle, min(steering_angle, self.max_steer_angle)))
        msg.speed = int(speed)
        self.motor_pub.publish(msg)
        
        # 디버그 정보 출력
        rospy.logdebug(f"제어 명령: 조향각={msg.angle}°, 속도={msg.speed}")


if __name__ == "__main__":
    try:
        controller = RelativePurePursuit()
        rospy.loginfo("상대위치 Pure Pursuit 시작")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Pure Pursuit 종료")