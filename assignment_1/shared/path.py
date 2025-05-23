class Path:
    def __init__(self):
        self.x = []       # 전역 x 좌표
        self.y = []       # 전역 y 좌표
        self.yaw = []     # 방향각 (rad), PurePursuit에서 yaw 계산 용
        self.ds = []      # 거리 차이, (x, y) 기반 간격

        # s, d Frenet 좌표계
        self.s = []
        self.d = []

###################################### 필요시 사용

        # 속도
        self.speed = []

        # 가속도
        self.acceleration = []

        # 곡률
        self.curvature = []

        # 종료 시간
        self.t = []