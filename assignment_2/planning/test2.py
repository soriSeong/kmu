# 로컬에서 경로 생성 확인하는 파일

import math
import numpy as np
import matplotlib.pyplot as plt

from reeds_shepp_path import (
    calc_optimal_path,
    pi_2_pi,
    STEP_SIZE
)

step_size = STEP_SIZE

# 공통 파라미터
max_curvature = 0.7
tol_xy = 0.1  # 위치 허용 오차 [m]
tol_yaw = math.radians(5)  # yaw 허용 오차 [rad]
max_dyaw = math.radians(45)

def check_and_plot(sx, sy, syaw, gx, gy, gyaw, case_id):
    print(f"\n=== CASE {case_id} ===")
    
    # 최적 경로 계산
    try:
        optimal = calc_optimal_path(sx, sy, syaw, gx, gy, gyaw, max_curvature, step_size)
        
        if optimal is None:
            print(" 경로를 찾지 못했습니다.")
            return
    except Exception as e:
        print(f" 경로 계산 중 오류 발생: {e}")
        import traceback
        traceback.print_exc()
        return
    
    print(f" start = ({sx:.1f}, {sy:.1f}, {math.degrees(syaw):.1f}°)")
    print(f"  goal = ({gx:.1f}, {gy:.1f}, {math.degrees(gyaw):.1f}°)")
    
    # 경로 가져오기
    x = optimal.x
    y = optimal.y
    yaw = optimal.yaw
    directions = optimal.directions
    
    # 3) 시작·종료 오차 체크
    dx0, dy0 = x[0]-sx, y[0]-sy
    dxf, dyf = x[-1]-gx, y[-1]-gy
    dyaw0 = pi_2_pi(yaw[0] - syaw)
    dyawf = pi_2_pi(yaw[-1] - gyaw)
    
    def deg(r): return math.degrees(r)
    
    if abs(dx0)>tol_xy or abs(dy0)>tol_xy:
        print(f" 시작 위치 오차: Δx={dx0:.3f}, Δy={dy0:.3f}")
    if abs(dxf)>tol_xy or abs(dyf)>tol_xy:
        print(f" 종료 위치 오차: Δx={dxf:.3f}, Δy={dyf:.3f}")
    if abs(dyaw0)>tol_yaw:
        print(f" 시작 yaw 오차: {deg(dyaw0):.2f}° (허용 {deg(tol_yaw):.1f}°)")
    if abs(dyawf)>tol_yaw:
        print(f" 종료 yaw 오차: {deg(dyawf):.2f}° (허용 {deg(tol_yaw):.1f}°)")
    
    # 4) 이격 & 급회전 체크
    for i in range(1, len(x)):
        dist = math.hypot(x[i]-x[i-1], y[i]-y[i-1])
        if abs(dist-step_size) > 0.02:
            print(f" 이격 오류 [{i}]: {dist:.3f}m (기대 {step_size}m)")
    
    for i in range(2, len(yaw)):
        ddyaw = abs(pi_2_pi(yaw[i]-2*yaw[i-1]+yaw[i-2]))
        if ddyaw > max_dyaw:
            print(f" 급회전 오류 [{i}]: Δyaw={deg(ddyaw):.2f}° (max {deg(max_dyaw):.0f}°)")
    
    # 5) 시각화 (축에 여유 추가)
    fig, ax = plt.subplots(figsize=(6,6))
    
    # 전진/후진에 따라 경로 색상 구분
    forward_x, forward_y = [], []
    backward_x, backward_y = [], []
    
    for i in range(len(x)):
        if directions[i] == 1:  # 전진
            forward_x.append(x[i])
            forward_y.append(y[i])
        else:  # 후진
            backward_x.append(x[i])
            backward_y.append(y[i])
    
    # 경로 그리기
    ax.plot(forward_x, forward_y, '-b', linewidth=2, label='전진')
    if backward_x:  # 후진 구간이 있는 경우
        ax.plot(backward_x, backward_y, '-r', linewidth=2, label='후진')
    
    # car 그리기
    L, W = 0.5, 0.3
    def draw_car(px, py, pyaw, color):
        corners = np.array([[ L/2,  W/2],
                            [ L/2, -W/2],
                            [-L/2, -W/2],
                            [-L/2,  W/2]])
        R = np.array([[math.cos(pyaw), -math.sin(pyaw)],
                      [math.sin(pyaw),  math.cos(pyaw)]])
        pts = corners.dot(R.T) + np.array([px, py])
        ax.fill(pts[:,0], pts[:,1], color=color, alpha=0.5)
    
    draw_car(x[0],  y[0],  yaw[0],  'green')
    draw_car(x[-1], y[-1], yaw[-1], 'red')
    print("실제 시작점:", x[0], y[0], math.degrees(yaw[0]))
    print("입력한 시작점:", sx, sy, math.degrees(syaw))
    
    # 가독성을 위해 방향 전환 지점 표시
    for i in range(1, len(directions)):
        if directions[i] != directions[i-1]:
            ax.plot(x[i], y[i], 'ko', markersize=5)  # 방향 전환 지점에 검은색 원 표시
    
    ax.set_aspect('equal')
    ax.margins(0.2)
    ax.grid(True)
    ax.legend()
    ax.set_title(f"Case {case_id}: {deg(syaw):.0f}°→{deg(gyaw):.0f}°")
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    plt.show()

if __name__ == '__main__':
    test_cases = [
        ( 0.0, 0.0,   math.radians(0),   5.0,  3.0,  math.radians(45)),
        ( 1.0, 2.0,   math.radians(90.0),  -5.5, 2.5,  math.radians(90.0)),
        ( 3.0, 1.0,   math.radians(0),   10.0,  4.0,  math.radians(0)),
        ( 2.5,-1.5,   math.radians(45),  -2.5,  2.5,  math.radians(30)),
        ( 1.5, 3.5,   math.radians(120),  5.5,  0.5,  math.radians(90))
    ]
    
    for idx, (sx, sy, syaw, gx, gy, gyaw) in enumerate(test_cases, start=1):
        check_and_plot(sx, sy, syaw, gx, gy, gyaw, idx)