#!/usr/bin/env python3
import threading
from time import sleep
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf
from .sub_function.motion import Motion
from controller.lane_driving import LaneDrivingController
from .optimal_frenet import *
import rospy

class MotionPlanner(threading.Thread):
    """
    Motion Planning 클래스
    부모 노드의 shared, plan, ego 객체를 참조해 주기마다 모션 제어 명령을 실행
    BehaviorPlanner가 설정한 motion_decision에 따라 적절한 Motion 호출
    """
    def __init__(self, parent, rate):
        """
        생성자
        parent: 상위 노드, shared, plan, ego를 참조하기 위해 사용
        rate: 실행 주기(Hz)
        super().__init__()를 통해 Thread 초기화한다.
        period 계산(1.0 / rate)하여 루프 대기 시간 설정,
        shared, plan, ego 객체를 로컬 변수에 저장,
        Motion 모듈 인스턴스 생성,
        lane_controller 인스턴스 생성
        """
        super().__init__()
        self.period = 1.0 / rate
        self.shared = parent.shared
        self.plan = parent.shared.plan
        self.ego = parent.shared.ego
        self.motion = Motion(self.shared)
        self.path_pub = rospy.Publisher("/frenet_path", Path, queue_size=1)
    
        self.lane_controller = LaneDrivingController(self.shared)

    def run(self):
        """
        스레드 진입점(run)
        무한 루프를 돌며 plan.motion_decision 값을 확인하고, 해당하는 motion 메서드를 호출한다.
        """
        while True:
            try:
                if self.plan.motion_decision == "go":
                    self.motion.go()
                
                elif self.plan.motion_decision == "traffic_light":
                    self.motion.traffic_light_decision()

                elif self.plan.motion_decision == "obs_small":
                    self.motion.obs_small()

                elif self.plan.motion_decision == "obs_big":
                    self.motion.obs_big()

            except IndexError:
                print("++++++++motion_planner+++++++++")
            sleep(self.period)
    
    def publish_frenet_path(self, frenet_path):
        """
        Frenet 경로를 ROS 메시지(Path)로 퍼블리시
        입력:
            - frenet_path: Path 객체(사용자 정의)에 포함된 x, y, yaw 리스트
        동작:
          1. Path() 메시지 객체 생성 및 Header 설정(ros 시간, frame_id="map")
          2. frenet_path.x, y, yaw를 순회하며 PoseStamped 메시지로 변환
             - position: x, y, z=0.0 설정
             - orientation: yaw를 Quaternion으로 변환(tf 사용)
             - pose.header.stamp, frame_id 설정
             - Path 메시지의 poses 리스트에 append
          3. 퍼블리셔(self.path_pub)를 통해 publish
        """
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for x, y, yaw in zip(frenet_path.x, frenet_path.y, frenet_path.yaw):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # yaw를 Quaternion으로 변환
            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            path_msg.poses.append(pose)

        # ROS 메시지를 퍼블리시
        self.path_pub.publish(path_msg)

    def generate_frenet_path(self):
        """
        Frenet 경로 생성
        동작:
          1. 현재 Ego 상태(ego.x, ego.y, ego.yaw, ego.speed)를 가져와 Vehicle 객체 생성
          2. 기준 경로 생성:
             a. generate_reference_path_from_vehicle(vehicle)를 호출해 mapx, mapy 리스트 획득
             b. calc_maps(mapx, mapy) 호출해 maps 생성
          3. 현재 차량 위치를 Frenet 좌표(s0, d0)로 변환: get_frenet(vehicle.x, vehicle.y, mapx, mapy)
          4. 장애물 위치 설정: obs_s = s0 + 15.0, obs_d = 0.0 배열 생성
          5. 왼쪽 회피 시도:
             a. frenet_optimal_planning_left(...) 호출
             b. 성공하면 publish_frenet_path()로 경로 퍼블리시 후 최적 경로 반환
          6. 왼쪽 회피 실패 시 예외 발생 → 오른쪽 회피 시도:
             a. frenet_optimal_planning_right(...) 호출
             b. 성공하면 publish_frenet_path() 후 최적 경로 반환
          7. 두 회피 모두 실패 시 None 반환
        반환:
          - 최적 Frenet 경로(Path 객체) 또는 None
        """
        ego = self.ego
        vehicle = Vehicle(ego.x, ego.y, ego.yaw, ego.speed)

        # 기준 경로 생성
        mapx, mapy = generate_reference_path_from_vehicle(vehicle)
        maps = calc_maps(mapx, mapy)

        # 현재 위치를 Frenet 좌표로 변환
        s0, d0 = get_frenet(vehicle.x, vehicle.y, mapx, mapy)
        obs_s = s0 + 15.0 
        obs_d = 0.0
        obs = np.array([[obs_s, obs_d]])

        # 왼쪽 회피 시도
        try:
            fplist, best_ind = frenet_optimal_planning_left(
                si=s0, si_d=ego.speed, si_dd=0,
                sf_d=TARGET_SPEED, sf_dd=0,
                di=d0, di_d=0, di_dd=0,
                df_d=0, df_dd=0,
                obs=obs, mapx=mapx, mapy=mapy, maps=maps,
                opt_d=d0
            )
            self.publish_frenet_path(fplist[best_ind])
            return fplist[best_ind]
        except:
            # 왼쪽 회피 실패 시 오른쪽 회피 시도
            print("좌측 회피 실패, 우측 시도")
            try:
                fplist, best_ind = frenet_optimal_planning_right(
                    si=s0, si_d=ego.speed, si_dd=0,
                    sf_d=TARGET_SPEED, sf_dd=0,
                    di=d0, di_d=0, di_dd=0,
                    df_d=0, df_dd=0,
                    obs=obs, mapx=mapx, mapy=mapy, maps=maps,
                    opt_d=d0
                )
                self.publish_frenet_path(fplist[best_ind])
                return fplist[best_ind]
            except:
                # 우측 회피도 실패한 경우
                print("우측 회피 실패")
                return None