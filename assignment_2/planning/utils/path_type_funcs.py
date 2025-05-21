import math
import numpy as np

class ReedsSheppPath:
    # lengths는 경로 유형에 따라 수가 달라짐 CSC면 3 CCSC는 4
    def __init__(self, x, y, yaw, directions, ctypes, lengths):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.directions = directions
        self.ctypes = ctypes # 경로 유형 
        self.length = sum([abs(l) for l in self.lengths]) # 경로 총 길이

TURNING_RADIUS = 0.686 # 차량 길이를 25cm로 추정 후 계산 R = L/tan(theta)
STEP_SIZE = 0.1  # 경로 생성를 0.1 간격으로 시행

# util func
def mod2pi(theta):
    """각도를 -pi에서 pi 사이 값으로 정규화"""
    v = np.mod(theta, np.copysign(2.0 * math.pi, theta))
    if v < -math.pi:
        v += 2.0 * math.pi
    elif v > math.pi:
        v -= 2.0 * math.pi
    return v

def pi_2_pi(angle):
    """각도를 -pi에서 pi 사이 값으로 정규화 (다른 방식)"""
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def calc_trig_funcs(alpha, beta, d):
    """삼각함수 계산"""
    sin_alpha = math.sin(alpha)
    cos_alpha = math.cos(alpha)
    sin_beta = math.sin(beta)
    cos_beta = math.cos(beta)
    
    return sin_alpha, cos_alpha, sin_beta, cos_beta

def calc_distance_and_angle(x, y):
    r = math.hypot(x, y)
    theta = math.atan2(y, x)
    return r, theta

# CSC 유형 LSL, LSR, RSR
# LSL (왼쪽, 직진, 왼쪽) 경로 계산
def LSL(x, y, phi, max_curvature):
    """LSL 경로 계산"""
    u, t = calc_distance_and_angle(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if 0.0 <= t <= math.pi:
        v = mod2pi(phi - t)
        if 0.0 <= v <= math.pi:
            return t, u, v
    else:
        return None, None, None

def LSR(x, y, phi, max_curvature):
    """LSR 경로 계산"""
    u1, t1 = calc_distance_and_angle(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u1 = u1 ** 2

    if u1 >= 4.0:
        u = math.sqrt(u1 - 4.0)
        theta = math.atan2(2.0, u)
        t = mod2pi(t1 + theta)
        v = mod2pi(t - phi)
        if t >= 0 and v >= 0:
            return t, u, v
    else:
        return None, None, None


# LRL (왼쪽, 오른쪽, 왼쪽) 경로 계산
# 시작 방향 phi
def L_X_R_X_L(x, y, phi, max_curvature):
    """LRL 경로 계산"""
    # 중간 변수 u, v를 계산
    u = x - math.sin(phi)
    v = y - 1 + math.cos(phi)
    u1, theta = calc_distance_and_angle(u, v)

    if u1 <= 4.0:
        A = math.acos(0.25 * u1)
        t = mod2pi(A + theta + math.pi/2)
        u = mod2pi(math.pi - 2*A)
        v = mod2pi(phi - t - u)
        return t, -u, v
    else:
        return None, None, None

def L_X_RL(x, y, phi, max_curvature):
    """L_X_RL 경로 계산"""
    # 중간 변수 u, v를 계산
    u = x - math.sin(phi)
    v = y - 1 + math.cos(phi)
    u1, theta = calc_distance_and_angle(u, v)

    if u1 <= 4.0:
        A = math.acos(0.25 * u1)
        t = mod2pi(A + theta + math.pi/2)
        u = mod2pi(math.pi - 2*A)
        v = mod2pi(-phi + t + u)
        return t, -u, -v
    else:
        return None, None, None

def LR_X_L(x, y, phi, max_curvature):
    """LR_X_L 경로 계산"""
    # 중간 변수 u, v를 계산
    u = x - math.sin(phi)
    v = y - 1 + math.cos(phi)
    u1, theta = calc_distance_and_angle(u, v)

    if u1 <= 4.0:
        A = mod2pi(2 * math.sin(u) / u1)
        u = math.acos(1 - u1**2 * 0.125)
        t = mod2pi(-A + theta + math.pi/2)
        v = mod2pi(-phi + t - u)
        return t, u, -v
    else:
        return None, None, None

#CCCC, CCSC 유형
def LR_X_LL(x, y, phi, max_curvature):
    """LR_X_LL 경로 계산"""
    # 중간 변수 u, v를 계산
    u = x + math.sin(phi)
    v = y - 1 - math.cos(phi)
    u1, theta = calc_distance_and_angle(u, v)

    if u1 <= 2.0:
        A = mod2pi(2 * math.sin(u) / u1)
        t = mod2pi(-A + theta + math.pi/2)
        u = mod2pi(A)
        v = mod2pi(phi - t + 2*u)
        if t >= 0 and u >= 0 and v >= 0:
            return t, u, -u, -v
    else:
        return None, None, None, None

def LR_X_L_X_R(x, y, phi, max_curvature):
    """LR_X_L_X_R 경로 계산"""
    # 중간 변수 u, v를 계산
    u = x + math.sin(phi)
    v = y - 1 - math.cos(phi)
    u1, theta = calc_distance_and_angle(u, v)
    u2 = (20 - u1**2) / 16

    if 0 <= u1 <= 1.0:
        u = math.cos(u2)
        A = math.asin(2 * math.sin(u) / u1)
        t = mod2pi(theta + A + math.pi/2)
        v = mod2pi(t - phi)
        if t >= 0 and v >= 0:
            return t, -u, -u, v
    else:
        return None, None, None, None

def L_X_R90SL(x, y, phi, max_curvature):
    u = x - math.sin(phi)
    v = y - 1 + math.cos(phi)
    u1, theta = calc_distance_and_angle(u, v)

    if u1 >= 2.0:
        u = math.sqrt(u1**2 - 4) - 2
        A = math.atan2(2, math.sqrt(u1**2 - 4))
        t = mod2pi(theta + A + math.pi/2)
        v = mod2pi(t - phi + math.pi/2)
        if (t >= 0) and (v >= 0):
            return t, -math.pi/2, -u, -v
    else:
        return None, None, None, None

def LSR90_X_R(x, y, phi, max_curvature):
    u = x - math.sin(phi)
    v = y - 1 + math.cos(phi)
    u1, theta = calc_distance_and_angle(u, v)

    if u1 >= 2.0:
        u = math.sqrt(u1**2 - 4) - 2
        A = math.atan2(math.sqrt(u1**2 - 4), 2)
        t = mod2pi(theta - A + math.pi/2)
        v = mod2pi(t - phi - math.pi/2)
        if (t >= 0) and (v >= 0):   
            return t, u, math.pi/2, -v
    else:
        return None, None, None, None

def L_X_R90SR(x, y, phi, max_curvature):
    u = x + math.sin(phi)
    v = y - 1 - math.cos(phi)
    u1, theta = calc_distance_and_angle(u, v)

    if u1 >= 2.0:
        t = mod2pi(theta + math.pi/2)
        u = u1 - 2
        v = mod2pi(phi - t - math.pi/2)
        if (t >= 0) and (v >= 0):
            return t, -math.pi/2, -u, -v
    else:
        return None, None, None, None

def LSL90_X_R(x, y, phi, max_curvature):
    u = x + math.sin(phi)
    v = y - 1 - math.cos(phi)
    u1, theta = calc_distance_and_angle(u, v)

    if u1 >= 2.0:
        t = mod2pi(theta)
        u = u1 - 2
        v = mod2pi(phi - t - math.pi/2)
        if (t >= 0) and (v >= 0):
            return t, u, math.pi/2, -v
    else:
        return None, None, None, None

#CCSCC 유형
def L_X_R90SL_X_R(x, y, phi, max_curvature):
    u = x + math.sin(phi)
    v = y - 1 - math.cos(phi)
    u1, theta = calc_distance_and_angle(u, v)

    if u1 >= 4.0:
        u = math.sqrt(u1**2 - 4) - 4
        A = math.atan2(2, math.sqrt(u1**2 - 4))
        t = mod2pi(theta + A + math.pi/2)
        v = mod2pi(t - phi)
        if (t >= 0) and (v >= 0):
            return t, -math.pi/2, -u, -math.pi/2, v
    else:
        return None, None, None, None, None
