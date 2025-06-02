class Ego:
    """
    차량의 현재 상태 저장하는 클래스
    """
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.speed = 0
        self.steer = 0