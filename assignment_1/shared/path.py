class Path:
    def __init__(self):
        self.x = []       # 전역 x 좌표
        self.y = []       # 전역 y 좌표
        self.yaw = []     # 방향각 (rad), PurePursuit에서 yaw 계산 용
        self.ds = []      # 거리 차이, (x, y) 기반 간격

