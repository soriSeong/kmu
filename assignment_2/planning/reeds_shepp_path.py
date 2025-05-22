import math
import numpy as np
from utils.path_enum import ReedsSheppPathType
from utils.path_type_funcs import (
    LSL, LSR, L_X_R_X_L,
    L_X_RL, LR_X_L, LR_X_LL,
    LR_X_L_X_R, L_X_R90SL, LSR90_X_R,
    L_X_R90SR, LSL90_X_R, L_X_R90SL_X_R,
    pi_2_pi
)

# 사용하는 모든 패턴 저장 논문을 참고해 작성했다. (가장 많이 사용하는 유형들)
PATTERNS = [
    (LSL,    ReedsSheppPathType.LSL,           ['L','S','L'], 3),
    (LSR,    ReedsSheppPathType.LSR,           ['L','S','R'], 3),
    (L_X_R_X_L,    ReedsSheppPathType.L_X_R_X_L,           ['L','X','R','X','L'],   3),
    (L_X_RL, ReedsSheppPathType.L_X_RL,        ['L','X','R','L'], 3),
    (LR_X_L, ReedsSheppPathType.LR_X_L,        ['L','R','X','L'],3),
    (LR_X_LL,ReedsSheppPathType.LR_X_LL,       ['L','R','X','L','L'], 4),
    (LR_X_L_X_R, ReedsSheppPathType.LR_X_L_X_R,['L','R','X','L','X','R'], 4),
    (L_X_R90SL,   ReedsSheppPathType.L_X_R90SL,['L','X','R','90','S','L'], 4),
    (LSR90_X_R,   ReedsSheppPathType.LSR90_X_R,['L','S','R','90','X','R'], 4),
    (L_X_R90SR,   ReedsSheppPathType.L_X_R90SR,['L','X','R','90','S','R'], 4),
    (LSL90_X_R,   ReedsSheppPathType.LSL90_X_R,['L','S','L','90','X','R'], 4),
    (L_X_R90SL_X_R, ReedsSheppPathType.L_X_R90SL_X_R,['L','X','R','90','S','L','X','R'], 5),
]

class PATH:
    """Reeds-Shepp 경로 클래스"""
    def __init__(self):
        self.x = []          # 경로 x 좌표
        self.y = []          # 경로 y 좌표
        self.yaw = []        # 경로 방향각
        self.directions = [] # 진행 방향 (1: 전진, -1: 후진)
        self.ctypes = []     # 경로 구간 유형 (L, S, R)
        self.lengths = []    # 각 구간 길이
        self.length = 0      # 총 경로 길이
        self.max_curvature = 0  # 최대 곡률
    
    def generate_path(self, start, goal):
        """
        시작점과 목표점 사이의 경로 생성
        """
        # 최대 곡률 설정 (회전 반경의 역수)
        max_curvature = 1.0
    
        # 시작점과 목표점에서 값 추출
        sx, sy, syaw = start
        gx, gy, gyaw = goal
    
        # 최적 경로 계산
        optimal_path = calc_optimal_path(sx, sy, syaw, gx, gy, gyaw, max_curvature, STEP_SIZE)
    
        if optimal_path is None:
            print("경로를 찾을 수 없습니다.")
            return [], [], []
    
        # 클래스 속성 업데이트
        self.x = optimal_path.x
        self.y = optimal_path.y
        self.yaw = optimal_path.yaw
        self.directions = optimal_path.directions
        self.ctypes = optimal_path.ctypes   
        self.lengths = optimal_path.lengths
        self.length = optimal_path.length
        self.max_curvature = max_curvature
        
        return optimal_path.x, optimal_path.y, optimal_path.yaw

TURNING_RADIUS = 0.686 # 차량 길이를 25cm로 추정 후 계산 R = L/tan(theta)
STEP_SIZE = 0.1  # 경로 생성를 0.1 간격으로 시행

def convert_to_radian(yaw):
    return yaw * np.pi / 180

def set_path(paths, lengths, ctypes, max_curvature, step_size, sx, sy, syaw):
    '''
    중복 검사 후 경로 추가 (좌표 계산 포함)
    '''
    path = PATH()
    path.ctypes = ctypes
    path.lengths = lengths
    path.length = sum(np.abs(lengths))

    # 중복 검사
    for i_path in paths:
        type_is_same = (i_path.ctypes == path.ctypes)
        length_is_close = (sum(np.abs(i_path.lengths)) - path.length) <= step_size
        if type_is_same and length_is_close:
            return paths  # 중복이면 그대로 반환

    # 경로가 너무 짧으면 추가하지 않음
    if path.length <= step_size:
        return paths

    # 로컬 좌표계에서 경로 생성
    x_local, y_local, yaw_local, directions = generate_local_course(
        lengths, ctypes, max_curvature, step_size
    )
    
    # 전역 좌표계로 변환
    x_global = []
    y_global = []
    yaw_global = []
    
    for i in range(len(x_local)):
        xx = x_local[i] * math.cos(syaw) - y_local[i] * math.sin(syaw) + sx
        yy = x_local[i] * math.sin(syaw) + y_local[i] * math.cos(syaw) + sy
        yawy = pi_2_pi(yaw_local[i] + syaw)
        
        x_global.append(xx)
        y_global.append(yy)
        yaw_global.append(yawy)
    
    # 경로 객체에 전역 좌표 저장
    path.x = x_global
    path.y = y_global
    path.yaw = yaw_global
    path.directions = directions
    path.max_curvature = max_curvature

    # 경로 추가
    paths.append(path)
    return paths

def generate_local_course(lengths, modes, max_curvature, step_size):
    """
    interpolate 함수 활용해 보간, 경로 생성 
    """
    # 포인트 수 예측
    total_length = sum([abs(l) for l in lengths])
    point_num = int(total_length / step_size) + len(lengths) + 3
    
    # 경로 저장 배열 초기화
    px = [0.0] * point_num
    py = [0.0] * point_num
    pyaw = [0.0] * point_num
    directions = [0] * point_num
    
    # 첫 번째 포인트 설정
    px[0], py[0], pyaw[0] = 0.0, 0.0, 0.0
    directions[0] = 1 if lengths[0] > 0.0 else -1
    
    # 현재 인덱스
    ind = 1
    
    # 현재 위치와 방향 (원점에서 시작)
    ox, oy, oyaw = 0.0, 0.0, 0.0
    
    # 각 세그먼트별 경로 생성
    for i, (mode, length) in enumerate(zip(modes, lengths)):
        # 스텝 크기 설정 (방향 고려)
        step = step_size if length > 0.0 else -step_size
        
        # 현재 세그먼트에서 이동할 거리
        dist = 0.0
        
        # 세그먼트 내 포인트 생성
        while abs(dist) < abs(length):
            # 다음 스텝으로 이동
            dist += step
            
            # 세그먼트 길이를 초과하지 않도록 보정
            if abs(dist) > abs(length):
                dist = length
            
            # interpolate 함수로 포인트 계산
            x, y, yaw, direction = interpolate(dist, length, mode, max_curvature, ox, oy, oyaw)
            
            # 위치와 방향 저장
            px[ind] = x
            py[ind] = y
            pyaw[ind] = yaw
            directions[ind] = direction
            
            ind += 1
            
            # 세그먼트의 끝에 도달하면 루프 종료
            if abs(dist - length) < 1e-10:
                break
        
        # 다음 세그먼트를 위한 시작점 업데이트
        ox, oy, oyaw = px[ind-1], py[ind-1], pyaw[ind-1]
    
    # 사용되지 않은 데이터 제거
    px = px[:ind]
    py = py[:ind]
    pyaw = pyaw[:ind]
    directions = directions[:ind]
    
    return px, py, pyaw, directions


def interpolate(dist, length, mode, max_curvature, ox, oy, origin_yaw):
    """
    특정 거리에서의 위치, 방향, 진행방향 계산
    """
    # 진행 방향 결정
    direction = 1 if length > 0.0 else -1
    
    # X는 후진 모드
    if mode == 'X':
        direction = -direction
        # 직선 보간
        x = ox + dist * math.cos(origin_yaw)
        y = oy + dist * math.sin(origin_yaw)
        yaw = origin_yaw
        return x, y, yaw, direction
    
    # 모드별 처리
    if mode == 'S':
        # 직선 보간
        x = ox + dist * math.cos(origin_yaw)
        y = oy + dist * math.sin(origin_yaw)
        yaw = origin_yaw
        
    elif mode == 'L':
        # 좌회전 보간
        ldx = math.sin(abs(dist) * max_curvature) / max_curvature
        ldy = (1.0 - math.cos(abs(dist) * max_curvature)) / max_curvature
        yaw = pi_2_pi(origin_yaw + dist * max_curvature)
        
        # 좌표계 변환 (로컬 → 전역)
        gdx = math.cos(-origin_yaw) * ldx + math.sin(-origin_yaw) * ldy
        gdy = -math.sin(-origin_yaw) * ldx + math.cos(-origin_yaw) * ldy
        
        x = ox + gdx
        y = oy + gdy
        
    elif mode == 'R':
        # 우회전 보간
        ldx = math.sin(abs(dist) * max_curvature) / max_curvature
        ldy = (1.0 - math.cos(abs(dist) * max_curvature)) / (-max_curvature)
        yaw = pi_2_pi(origin_yaw - dist * max_curvature)
        
        # 좌표계 변환 (로컬 → 전역)
        gdx = math.cos(-origin_yaw) * ldx + math.sin(-origin_yaw) * ldy
        gdy = -math.sin(-origin_yaw) * ldx + math.cos(-origin_yaw) * ldy
        
        x = ox + gdx
        y = oy + gdy
        
    else:
        x = ox
        y = oy
        yaw = origin_yaw

    return x, y, yaw, direction


def calc_all_paths(sx, sy, syaw, gx, gy, gyaw, max_curvature, step_size=STEP_SIZE):
    """모든 가능한 경로 계산"""
    paths = []

    # 좌표 변환
    dx = gx - sx
    dy = gy - sy

    # 시작 위치 기준으로 좌표계 변환
    c, s = math.cos(-syaw), math.sin(-syaw)

    # 변환된 좌표계에서의 목표 위치
    x = dx * c - dy * s
    y = dx * s + dy * c

    # 목표 방향 변환
    phi = gyaw - syaw
    phi = pi_2_pi(phi)

    # 모든 패턴을 직접 순회
    for pattern_func, path_type, ctypes, param_count in PATTERNS:
        # 경로 파라미터 계산
        params = pattern_func(x, y, phi, max_curvature)

        if params is not None and params[0] is not None:
            # 파라미터 수만큼 길이 추출
            lengths = list(params[:param_count])

            # 경로 추가
            paths = set_path(paths, lengths, ctypes, max_curvature, step_size, sx, sy, syaw)

    return paths

def calc_optimal_path(sx, sy, syaw, gx, gy, gyaw, max_curvature, step_size=STEP_SIZE):
    """
    시작점과 끝점 사이의 최적 경로 계산
    """
    # 모든 경로 계산
    paths = calc_all_paths(sx, sy, syaw, gx, gy, gyaw, max_curvature, step_size)
    
    if not paths:
        return None
    
    # 전진 경로 우선 선택 (선택적)
    forward_only = [p for p in paths if all(l >= 0 for l in p.lengths)]
    if forward_only:
        paths = forward_only
    
    # 경로 길이 기준으로 정렬
    paths.sort(key=lambda p: p.length)
    
    # 최단 경로 선택
    optimal_path = paths[0]
    
    # 마지막 포인트를 목표 좌표로 정확히 설정
    if len(optimal_path.x) > 0:
        optimal_path.x[-1] = gx
        optimal_path.y[-1] = gy
        optimal_path.yaw[-1] = gyaw
    
    return optimal_path


def calc_curvature(x, y, yaw):
    """
    경로의 곡률 계산
    """
    x = np.array(x)
    y = np.array(y)
    yaw = np.array(yaw)
    
    n = len(x)

    curvatures = [0.0] # 첫 번째 점은 곡률이 0이다.

    for i in range(1, n - 1):
        # 1차 도함수
        dx = (x[i+1] - x[i-1]) / 2.0
        dy = (y[i+1] - y[i-1]) / 2.0

        # 2차 도함수
        ddx = x[i+1] - 2 * x[i] + x[i-1]
        ddy = y[i+1] - 2 * y[i] + y[i-1]

        # 분자: |c' x c''| = dx*ddy - dy*ddx
        numerator = abs(dx * ddy - dy * ddx)

        # 분모: |c'|^3
        denominator = (dx ** 2 + dy ** 2) ** 1.5

        # 곡률
        try:
            k = numerator / denominator
        except ZeroDivisionError:
            k = 0.0

        curvatures.append(k)

    curvatures.append(0.0)  # 마지막 점 곡률 계산 불가

    return curvatures

def check_path(x, y, yaw,
               sx, sy, syaw,
               gx, gy, gyaw,
               step_size=0.1,
               max_dyaw=np.radians(45),
               tol=0.1):
    """
    리스트 기반 경로 유효성 검사 함수
    """
    assert abs(x[0] - sx) <= tol, f"시작 x 불일치: {x[0]} vs {sx}"
    assert abs(y[0] - sy) <= tol, f"시작 y 불일치: {y[0]} vs {sy}"
    assert abs(pi_2_pi(yaw[0] - syaw)) <= tol, f"시작 yaw 불일치"

    assert abs(x[-1] - gx) <= tol, f"종점 x 불일치: {x[-1]} vs {gx}"
    assert abs(y[-1] - gy) <= tol, f"종점 y 불일치: {y[-1]} vs {gy}"
    assert abs(pi_2_pi(yaw[-1] - gyaw)) <= tol, f"종점 yaw 불일치"

    for i in range(1, len(x)):
        dx = x[i] - x[i-1]
        dy = y[i] - y[i-1]
        dist = math.hypot(dx, dy)
        assert abs(dist - step_size) <= 0.02, f"[이격 오류] {i}: {dist:.3f}m"

    for i in range(2, len(yaw)):
        dyaw1 = pi_2_pi(yaw[i-1] - yaw[i-2])
        dyaw2 = pi_2_pi(yaw[i]   - yaw[i-1])
        ddyaw = abs(dyaw2 - dyaw1)
        assert ddyaw < max_dyaw, f"[급회전 오류] {i}: Δyaw = {math.degrees(ddyaw):.2f}°"

    print("경로 유효성 검사 통과")