import numpy as np
from math import atan2, cos, sin, sqrt
from copy import deepcopy
import math

# 파라미터 설정
V_MAX = 100      # 최대 속도 [m/s]
ACC_MAX = 100    # 최대 가속도 [m/s²]
K_MAX = 100      # 최대 곡률 [1/m]

TARGET_SPEED = 10  # 목표 속도 [m/s]
LANE_WIDTH = 4     # 차선 폭 [m]

COL_CHECK = 0.25   # 충돌 검사 거리 [m]

MIN_T = 2    # 최소 종료 시간 [s]
MAX_T = 2    # 최대 종료 시간 [s]
DT_T = 0.5   # 종료 시간 간격 [s]
DT = 0.05    # 업데이트 시간 간격 [s]

# 비용 가중치
K_J = 0.1    # 저크 가중치
K_T = 0.1    # 시간 가중치
K_D = 1.0    # 일관성 가중치
K_V = 1.0    # 목표 속도 가중치
K_LAT = 1.0  # 횡방향 가중치
K_LON = 1.0  # 종방향 가중치

TARGET_SPEED_DEFAULT = 10

# 차선 변경을 위한 횡방향 오프셋 설정
DF_SET_LEFT = np.array([LANE_WIDTH/2])
DF_SET_RIGHT = np.array([-LANE_WIDTH/2])

#############################################################################
# 유틸리티 함수: 좌표 변환 및 Frenet 관련
#############################################################################
class Vehicle:
    def __init__(self, initial_x=0, initial_y=0, initial_yaw=0, initial_v=0):
        self.x = initial_x
        self.y = initial_y
        self.yaw = initial_yaw
        self.v = initial_v
        self.direct = 1  # 1: 전진, -1: 후진
        
        # 상수
        self.dt = 0.1  # 시간 간격
        self.WB = 2.7  # 휠베이스 [m]

    def update(self, a, delta, direct):
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += self.v / self.WB * math.tan(delta) * self.dt
        self.direct = direct
        self.v = self.direct * a 

    def get_position(self):
        """현재 차량 위치 반환"""
        return self.x, self.y, self.yaw

def generate_reference_path_from_vehicle(vehicle, distance=50):
    """Vehicle의 현재 위치에서 경로 생성, mapx, mapy 초기화"""
    # 현재 차량 위치
    current_x, current_y, current_yaw = vehicle.get_position()
    
    # 직선 경로 생성 (mapx, mapy)
    mapx = []  # 경로의 x 좌표들
    mapy = []  # 경로의 y 좌표들
    
    for i in range(distance):   
        mapx.append(current_x + i * math.cos(current_yaw))
        mapy.append(current_y + i * math.sin(current_yaw))
    
    return mapx, mapy

def next_waypoint(x, y, mapx, mapy):
    """가장 가까운 다음 waypoint 인덱스를 찾는 함수"""
    closest_wp = get_closest_waypoints(x, y, mapx, mapy)

    map_vec = [mapx[closest_wp + 1] - mapx[closest_wp], mapy[closest_wp + 1] - mapy[closest_wp]]
    ego_vec = [x - mapx[closest_wp], y - mapy[closest_wp]]

    direction = np.sign(np.dot(map_vec, ego_vec))

    if direction >= 0:
        next_wp = closest_wp + 1
    else:
        next_wp = closest_wp

    return next_wp

def calc_maps(mapx, mapy):
    """경로의 누적 거리를 계산하는 함수"""
    maps = [0.0]
    for i in range(1, len(mapx)):
        dx = mapx[i] - mapx[i-1]
        dy = mapy[i] - mapy[i-1]
        maps.append(maps[-1] + np.sqrt(dx**2 + dy**2))
    return np.array(maps)

def get_closest_waypoints(x, y, mapx, mapy):
    """가장 가까운 waypoint 인덱스를 찾는 함수"""
    min_len = 1e10
    closest_wp = 0

    for i in range(len(mapx)):
        _mapx = mapx[i]
        _mapy = mapy[i]
        dist = get_dist(x, y, _mapx, _mapy)

        if dist < min_len:
            min_len = dist
            closest_wp = i

    return closest_wp

def get_dist(x, y, _x, _y):
    """두 점 사이의 거리를 계산하는 함수"""
    return np.sqrt((x - _x)**2 + (y - _y)**2)

def get_frenet(x, y, mapx, mapy):
    """직교좌표를 Frenet 좌표로 변환하는 함수"""
    next_wp = next_waypoint(x, y, mapx, mapy)
    prev_wp = next_wp - 1

    n_x = mapx[next_wp] - mapx[prev_wp]
    n_y = mapy[next_wp] - mapy[prev_wp]
    x_x = x - mapx[prev_wp]
    x_y = y - mapy[prev_wp]

    # 정사영 비율 계산
    proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y)
    proj_x = proj_norm * n_x
    proj_y = proj_norm * n_y

    # Frenet d 좌표 계산
    frenet_d = get_dist(x_x, x_y, proj_x, proj_y)

    ego_vec = [x - mapx[prev_wp], y - mapy[prev_wp], 0]
    map_vec = [n_x, n_y, 0]
    d_cross = np.cross(ego_vec, map_vec)
    if d_cross[-1] > 0:
        frenet_d = -frenet_d

    # Frenet s 좌표 계산
    frenet_s = 0
    for i in range(prev_wp):
        frenet_s = frenet_s + get_dist(mapx[i], mapy[i], mapx[i+1], mapy[i+1])

    frenet_s = frenet_s + get_dist(0, 0, proj_x, proj_y)

    return frenet_s, frenet_d

def get_cartesian(s, d, mapx, mapy, maps):
    """Frenet 좌표를 직교좌표로 변환하는 함수"""
    prev_wp = 0

    # s값을 도로 길이 내로 조정
    s = np.mod(s, maps[-2])

    # s가 속하는 구간 찾기
    while(s > maps[prev_wp+1]) and (prev_wp < len(maps)-2):
        prev_wp = prev_wp + 1

    next_wp = np.mod(prev_wp+1, len(mapx))

    dx = (mapx[next_wp] - mapx[prev_wp])
    dy = (mapy[next_wp] - mapy[prev_wp])

    heading = np.arctan2(dy, dx)

    seg_s = s - maps[prev_wp]

    seg_x = mapx[prev_wp] + seg_s * np.cos(heading)
    seg_y = mapy[prev_wp] + seg_s * np.sin(heading)

    perp_heading = heading + 90 * np.pi / 180
    x = seg_x + d * np.cos(perp_heading)
    y = seg_y + d * np.sin(perp_heading)

    return x, y, heading

#############################################################################
# 다항식 궤적 생성 클래스
#############################################################################

class QuinticPolynomial:
    """5차 다항식을 이용한 횡방향 궤적 생성"""
    
    def __init__(self, xi, vi, ai, xf, vf, af, T):
        self.a0 = xi
        self.a1 = vi
        self.a2 = 0.5 * ai

        A = np.array([[T**3, T**4, T**5],
                      [3*T**2, 4*T**3, 5*T**4],
                      [6*T, 12*T**2, 20*T**3]])
        b = np.array([xf - self.a0 - self.a1*T - self.a2*T**2,
                      vf - self.a1 - 2*self.a2*T,
                      af - 2*self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_pos(self, t):
        """위치 계산"""
        x = self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3 + self.a4*t**4 + self.a5*t**5
        return x

    def calc_vel(self, t):
        """속도 계산"""
        v = self.a1 + 2*self.a2*t + 3*self.a3*t**2 + 4*self.a4*t**3 + 5*self.a5*t**4
        return v

    def calc_acc(self, t):
        """가속도 계산"""
        a = 2*self.a2 + 6*self.a3*t + 12*self.a4*t**2 + 20*self.a5*t**3
        return a

    def calc_jerk(self, t):
        """저크 계산"""
        j = 6*self.a3 + 24*self.a4*t + 60*self.a5*t**2
        return j

class QuarticPolynomial:
    """4차 다항식을 이용한 종방향 궤적 생성"""
    
    def __init__(self, xi, vi, ai, vf, af, T):
        self.a0 = xi
        self.a1 = vi
        self.a2 = 0.5 * ai

        A = np.array([[3*T**2, 4*T**3],
                      [6*T, 12*T**2]])
        b = np.array([vf - self.a1 - 2*self.a2*T,
                      af - 2*self.a2])

        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_pos(self, t):
        """위치 계산"""
        x = self.a0 + self.a1*t + self.a2*t**2 + self.a3*t**3 + self.a4*t**4
        return x

    def calc_vel(self, t):
        """속도 계산"""
        v = self.a1 + 2*self.a2*t + 3*self.a3*t**2 + 4*self.a4*t**3
        return v

    def calc_acc(self, t):
        """가속도 계산"""
        a = 2*self.a2 + 6*self.a3*t + 12*self.a4*t**2
        return a

    def calc_jerk(self, t):
        """저크 계산"""
        j = 6*self.a3 + 24*self.a4*t
        return j

#############################################################################
# Frenet 경로 클래스
#############################################################################

class FrenetPath:
    """Frenet 프레임에서의 경로 정보를 저장하는 클래스"""
    
    def __init__(self):
        # 시간
        self.t = []

        # Frenet 프레임에서의 횡방향 궤적
        self.d = []      # 횡방향 위치
        self.d_d = []    # 횡방향 속도
        self.d_dd = []   # 횡방향 가속도
        self.d_ddd = []  # 횡방향 저크

        # Frenet 프레임에서의 종방향 궤적
        self.s = []      # 종방향 위치
        self.s_d = []    # 종방향 속도
        self.s_dd = []   # 종방향 가속도
        self.s_ddd = []  # 종방향 저크

        # 비용
        self.c_lat = 0.0  # 횡방향 비용
        self.c_lon = 0.0  # 종방향 비용
        self.c_tot = 0.0  # 전체 비용

        # 전역 프레임에서의 궤적
        self.x = []       # x 좌표
        self.y = []       # y 좌표
        self.yaw = []     # 방향각
        self.ds = []      # 구간 거리
        self.kappa = []   # 곡률

#############################################################################
# 경로 생성 함수들
#############################################################################

def calc_frenet_paths_left(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d):
    """좌측 차선 변경을 위한 Frenet 경로들을 생성하는 함수"""
    frenet_paths = []

    for df in DF_SET_LEFT:
        for T in np.arange(MIN_T, MAX_T + DT_T, DT_T):
            fp = FrenetPath()
            lat_traj = QuinticPolynomial(di, di_d, di_dd, df, df_d, df_dd, T)
            
            fp.t = [t for t in np.arange(0.0, T, DT)]
            fp.d = [lat_traj.calc_pos(t) for t in fp.t]
            fp.d_d = [lat_traj.calc_vel(t) for t in fp.t]
            fp.d_dd = [lat_traj.calc_acc(t) for t in fp.t]
            fp.d_ddd = [lat_traj.calc_jerk(t) for t in fp.t]

            # 종방향 계획 (속도 유지)
            tfp = deepcopy(fp)
            lon_traj = QuarticPolynomial(si, si_d, si_dd, sf_d, sf_dd, T)

            tfp.s = [lon_traj.calc_pos(t) for t in fp.t]
            tfp.s_d = [lon_traj.calc_vel(t) for t in fp.t]
            tfp.s_dd = [lon_traj.calc_acc(t) for t in fp.t]
            tfp.s_ddd = [lon_traj.calc_jerk(t) for t in fp.t]

            # 경로 연장 (T < MAX_T인 경우)
            for _t in np.arange(T, MAX_T + 3*DT, DT):
                tfp.t.append(_t)
                tfp.d.append(tfp.d[-1])  # 횡방향 위치 유지
                _s = tfp.s[-1] + tfp.s_d[-1] * DT  # 일정 속도로 직진
                tfp.s.append(_s)

                tfp.s_d.append(tfp.s_d[-1])
                tfp.s_dd.append(tfp.s_dd[-1])
                tfp.s_ddd.append(tfp.s_ddd[-1])

                tfp.d_d.append(tfp.d_d[-1])
                tfp.d_dd.append(tfp.d_dd[-1])
                tfp.d_ddd.append(tfp.d_ddd[-1])

            # 비용 계산
            J_lat = sum(np.power(tfp.d_ddd, 2))  # 횡방향 저크
            J_lon = sum(np.power(tfp.s_ddd, 2))  # 종방향 저크

            d_diff = (tfp.d[-1] - opt_d) ** 2  # 일관성 비용
            v_diff = (TARGET_SPEED - tfp.s_d[-1]) ** 2  # 목표 속도 비용

            tfp.c_lat = K_J * J_lat + K_T * T + K_D * d_diff
            tfp.c_lon = K_J * J_lon + K_T * T + K_V * v_diff
            tfp.c_tot = K_LAT * tfp.c_lat + K_LON * tfp.c_lon

            frenet_paths.append(tfp)

    return frenet_paths

def calc_frenet_paths_right(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d):
    """우측 차선 변경을 위한 Frenet 경로들을 생성하는 함수"""
    frenet_paths = []

    for df in DF_SET_RIGHT:
        for T in np.arange(MIN_T, MAX_T + DT_T, DT_T):
            fp = FrenetPath()
            lat_traj = QuinticPolynomial(di, di_d, di_dd, df, df_d, df_dd, T)

            fp.t = [t for t in np.arange(0.0, T, DT)]
            fp.d = [lat_traj.calc_pos(t) for t in fp.t]
            fp.d_d = [lat_traj.calc_vel(t) for t in fp.t]
            fp.d_dd = [lat_traj.calc_acc(t) for t in fp.t]
            fp.d_ddd = [lat_traj.calc_jerk(t) for t in fp.t]

            # 종방향 계획 (속도 유지)
            tfp = deepcopy(fp)
            lon_traj = QuarticPolynomial(si, si_d, si_dd, sf_d, sf_dd, T)

            tfp.s = [lon_traj.calc_pos(t) for t in fp.t]
            tfp.s_d = [lon_traj.calc_vel(t) for t in fp.t]
            tfp.s_dd = [lon_traj.calc_acc(t) for t in fp.t]
            tfp.s_ddd = [lon_traj.calc_jerk(t) for t in fp.t]

            # 경로 연장 (T < MAX_T인 경우)
            for _t in np.arange(T, MAX_T + 3*DT, DT):
                tfp.t.append(_t)
                tfp.d.append(tfp.d[-1])  # 횡방향 위치 유지
                _s = tfp.s[-1] + tfp.s_d[-1] * DT  # 일정 속도로 직진
                tfp.s.append(_s)

                tfp.s_d.append(tfp.s_d[-1])
                tfp.s_dd.append(tfp.s_dd[-1])
                tfp.s_ddd.append(tfp.s_ddd[-1])

                tfp.d_d.append(tfp.d_d[-1])
                tfp.d_dd.append(tfp.d_dd[-1])
                tfp.d_ddd.append(tfp.d_ddd[-1])

            # 비용 계산
            J_lat = sum(np.power(tfp.d_ddd, 2))  # 횡방향 저크
            J_lon = sum(np.power(tfp.s_ddd, 2))  # 종방향 저크

            d_diff = (tfp.d[-1] - opt_d) ** 2  # 일관성 비용
            v_diff = (TARGET_SPEED - tfp.s_d[-1]) ** 2  # 목표 속도 비용

            tfp.c_lat = K_J * J_lat + K_T * T + K_D * d_diff
            tfp.c_lon = K_J * J_lon + K_T * T + K_V * v_diff
            tfp.c_tot = K_LAT * tfp.c_lat + K_LON * tfp.c_lon

            frenet_paths.append(tfp)

    return frenet_paths

def calc_global_paths(fplist, mapx, mapy, maps):
    """Frenet 경로를 전역 좌표계로 변환하는 함수"""
    for fp in fplist:
        for i in range(len(fp.s)):
            _s = fp.s[i]
            _d = fp.d[i]
            _x, _y, _ = get_cartesian(_s, _d, mapx, mapy, maps)
            fp.x.append(_x)
            fp.y.append(_y)

        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(np.arctan2(dy, dx))
            fp.ds.append(np.hypot(dx, dy))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # 곡률 계산
        for i in range(len(fp.yaw) - 1):
            yaw_diff = fp.yaw[i + 1] - fp.yaw[i]
            yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
            fp.kappa.append(yaw_diff / fp.ds[i])

    return fplist

def collision_check(fp, obs, mapx, mapy, maps):
    """장애물과의 충돌을 검사하는 함수"""
    for i in range(len(obs[:, 0])):
        # 장애물의 위치 (x,y)
        obs_xy = get_cartesian(obs[i, 0], obs[i, 1], mapx, mapy, maps)

        d = [((_x - obs_xy[0]) ** 2 + (_y - obs_xy[1]) ** 2)
             for (_x, _y) in zip(fp.x, fp.y)]

        collision = any([di <= COL_CHECK ** 2 for di in d])

        if collision:
            return True

    return False

def check_path(fplist, obs, mapx, mapy, maps):
    """생성된 경로들의 유효성을 검사하는 함수"""
    ok_ind = []
    for i, _path in enumerate(fplist):
        acc_squared = [(abs(a_s**2 + a_d**2)) for (a_s, a_d) in zip(_path.s_dd, _path.d_dd)]
        
        if any([v > V_MAX for v in _path.s_d]):  # 최대 속도 검사
            continue
        elif any([acc > ACC_MAX**2 for acc in acc_squared]):  # 최대 가속도 검사
            continue
        elif any([abs(kappa) > K_MAX for kappa in fplist[i].kappa]):  # 최대 곡률 검사
            continue
        elif collision_check(_path, obs, mapx, mapy, maps):  # 충돌 검사
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]

def frenet_optimal_planning_left(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, mapx, mapy, maps, opt_d):
    """좌측 차선 변경을 위한 최적 경로를 계획하는 함수"""
    fplist = calc_frenet_paths_left(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d)
    fplist = calc_global_paths(fplist, mapx, mapy, maps)
    fplist = check_path(fplist, obs, mapx, mapy, maps)
    
    # 최소 비용 경로 찾기
    min_cost = float("inf")
    opt_traj = None
    opt_ind = 0
    for fp in fplist:
        if min_cost >= fp.c_tot:
            min_cost = fp.c_tot
            opt_traj = fp
            _opt_ind = opt_ind
        opt_ind += 1

    try:
        _opt_ind
    except NameError:
        print("해결책이 없습니다!")

    return fplist, _opt_ind

def frenet_optimal_planning_right(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, obs, mapx, mapy, maps, opt_d):
    """우측 차선 변경을 위한 최적 경로를 계획하는 함수"""
    fplist = calc_frenet_paths_right(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d)
    fplist = calc_global_paths(fplist, mapx, mapy, maps)
    fplist = check_path(fplist, obs, mapx, mapy, maps)
    
    # 최소 비용 경로 찾기
    min_cost = float("inf")
    opt_traj = None
    opt_ind = 0
    for fp in fplist:
        if min_cost >= fp.c_tot:
            min_cost = fp.c_tot
            opt_traj = fp
            _opt_ind = opt_ind
        opt_ind += 1

    try:
        _opt_ind
    except NameError:
        print("해결책이 없습니다!")

    return fplist, _opt_ind

############### 테스팅 ##################

import matplotlib.pyplot as plt

def generate_obstacle(vehicle, dist_ahead=15):
    obs_x = vehicle.x + dist_ahead * np.cos(vehicle.yaw)
    obs_y = vehicle.y + dist_ahead * np.sin(vehicle.yaw)
    return np.array([obs_x, obs_y])

def test_obstacle_avoidance_left():
    vehicle = Vehicle(initial_x=0, initial_y=0, initial_yaw=0, initial_v=TARGET_SPEED)
    trajectory = []
    obstacle_log = []

    for step in range(50):
        # 1. 차량 위치 갱신
        vehicle.update(a=TARGET_SPEED, delta=0.0, direct=1)
        trajectory.append((vehicle.x, vehicle.y))

        # 2. 기준 경로 생성
        mapx, mapy = generate_reference_path_from_vehicle(vehicle)
        maps = calc_maps(mapx, mapy)

        # 3. 현재 차량 위치 Frenet 변환
        s0, d0 = get_frenet(vehicle.x, vehicle.y, mapx, mapy)

        # 4. 장애물: 차량 기준 15m 앞
        obs_s = s0 + 15.0
        obs_d = 0.0
        obs = np.array([[obs_s, obs_d]])
        obs_x, obs_y, _ = get_cartesian(obs_s, obs_d, mapx, mapy, maps)
        obstacle_log.append((obs_x, obs_y))

        # 5. 회피 경로 생성 #left right로 바꾸면 오른쪽으로 잘 생성됨
        fplist, best_ind = frenet_optimal_planning_left(
            si=s0, si_d=vehicle.v, si_dd=0,
            sf_d=TARGET_SPEED, sf_dd=0,
            di=d0, di_d=0, di_dd=0,
            df_d=0, df_dd=0,
            obs=obs, mapx=mapx, mapy=mapy, maps=maps,
            opt_d=d0
        )

        # 6. 경로 출력 및 시각화
        print(f"[{step}] Vehicle Pos: ({vehicle.x:.2f}, {vehicle.y:.2f})")
        if fplist:
            opt_path = fplist[best_ind]
            print(f" → Path End: ({opt_path.x[-1]:.2f}, {opt_path.y[-1]:.2f})")
            plt.plot(mapx, mapy, 'k--', alpha=0.3)
            plt.plot(opt_path.x, opt_path.y, 'b-', linewidth=2)
        else:
            print(" → 유효한 회피 경로 없음")

        plt.scatter(vehicle.x, vehicle.y, color='red', label="Vehicle" if step == 0 else "")
        plt.scatter(obs_x, obs_y, color='orange', marker='X', s=100, label="Obstacle" if step == 0 else "")

        plt.axis("equal")
        plt.title("Frenet Planning: Step-by-Step Obstacle Avoidance")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.grid(True)
        plt.legend()

        plt.pause(0.3)  # 0.3초 간 멈춤
    
    plt.ioff()
    plt.show()

# 메인 실행
def main():
    test_obstacle_avoidance_left()

if __name__ == "__main__":
    main()
