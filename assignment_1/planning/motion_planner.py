import threading
from time import sleep
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf
from .sub_function.motion import Motion

from .optimal_frenet import (
    Vehicle, generate_reference_path_from_vehicle, calc_maps, get_frenet,
    frenet_optimal_planning_left, frenet_optimal_planning_right, TARGET_SPEED
)

class MotionPlanner(threading.Thread):
    def __init__(self, parent, rate):
        super().__init__()
        self.period = 1.0 / rate
        self.shared = parent.shared
        self.plan = parent.shared.plan
        self.ego = parent.shared.ego 
        self.motion = Motion()
        self.path_pub = rospy.Publisher("/frenet_path", Path, queue_size=1)

    def run(self):
        while True:
            try:
                if self.plan.motion_decision == "go":
                    self.motion.go()
                
                elif self.plan.motion_decision == "traffic_light":
                    self.motion.traffic_light()
                # 라바콘
                elif self.plan.motion_decision == "obs_small":
                    self.motion.obs_small()
                # 차량 회피
                elif self.plan.motion_decision == "obs_big":
                    optimal_path = self.generate_frenet_path()
                    self.shared.local_path = optimal_path  # 생성된 경로 저장
                    self.motion.obs_big() # 회피 제어

            except IndexError:
                print("++++++++motion_planner+++++++++")

            sleep(self.period)
    
    def publish_frenet_path(self, frenet_path):
        '''
        Frenet 경로를 ROS 메시지로 퍼블리쉬
        '''
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        # (x, y, yaw) 좌표계를 PoseStamped 메시지로 변환
        for x, y, yaw in zip(frenet_path.x, frenet_path.y, frenet_path.yaw):
            pose = PoseStamped()
            # 헤더를 현재시간, "map"으로 설정
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # yaw를 Quaternion으로 변환 tf를 사용한다. 
            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            # PoseStamped 메시지를 path_msg의 poses에 추가
            path_msg.poses.append(pose)

        # ROS 메시지를 퍼블리쉬
        self.path_pub.publish(path_msg)

    def generate_frenet_path(self):
        """Frenet 경로 생성"""
        ego = self.ego
        vehicle = Vehicle(ego.x, ego.y, ego.yaw, ego.speed)

        # 기준 경로
        mapx, mapy = generate_reference_path_from_vehicle(vehicle)
        maps = calc_maps(mapx, mapy)

        # 현재 차량 위치 -> Frenet 변환
        s0, d0 = get_frenet(vehicle.x, vehicle.y, mapx, mapy)
        obs_s = s0 + 15.0 
        obs_d = 0.0
        obs = np.array([[obs_s, obs_d]])

        # 좌측 회피 실패시 우측 회피 시도
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
                print("우측 회피 실패")
                return None
