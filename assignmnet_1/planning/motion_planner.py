import threading
from time import sleep
from .sub_function.motion import Motion
from numpy import dot
from math import cos, sin

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
        self.ego = parent.shared.ego  # ego 정보 가져오기
        self.motion = Motion()

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
    
    def generate_frenet_path(self):
        """장애물 회피용 Frenet 경로 생성"""
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
            return fplist[best_ind]
        except:
            print("좌측 회피 실패, 우측 시도")
            fplist, best_ind = frenet_optimal_planning_right(
                si=s0, si_d=ego.speed, si_dd=0,
                sf_d=TARGET_SPEED, sf_dd=0,
                di=d0, di_d=0, di_dd=0,
                df_d=0, df_dd=0,
                obs=obs, mapx=mapx, mapy=mapy, maps=maps,
                opt_d=d0
            )
            return fplist[best_ind]

