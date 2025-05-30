import pygame
import math
import numpy as np
from planning.reeds_shepp_path import calc_optimal_path, STEP_SIZE  # 원래대로 복구

class PygamePurePursuitTest:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((1000, 800))
        pygame.display.set_caption("Pure Pursuit 테스트")
        self.clock = pygame.time.Clock()
        
        # 색상 정의
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.GREEN = (0, 255, 0)
        self.RED = (255, 0, 0)
        self.BLUE = (0, 0, 255)
        self.YELLOW = (255, 255, 0)
        self.GRAY = (128, 128, 128)
        
        # 좌표 변환 (화면 중앙을 원점으로)
        self.center_x = 500
        self.center_y = 400
        self.scale = 100  # 1미터 = 100픽셀
        
        # Pure Pursuit 파라미터 (실제 코드와 동일)
        self.wheelbase = 0.25
        self.lookahead_distance = 0.3
        self.lookahead_gain = 0.1
        self.min_lookahead = 0.2
        self.max_lookahead = 0.6
        self.base_speed = 8
        self.goal_tolerance = 0.05
        
        # 시뮬레이션 상태
        self.path_points = []
        self.current_path_index = 0
        self.virtual_position = [0, 0, 0]  # 가상 현재 위치
        self.goal_position = None
        self.lookahead_point = None
        self.steering_angle = 0
        
        # 자동 시뮬레이션
        self.auto_mode = True
        self.simulation_speed = 1  # 시뮬레이션 속도 배율
        
    def world_to_screen(self, x, y):
        """월드 좌표를 화면 좌표로 변환"""
        screen_x = int(self.center_x + x * self.scale)
        screen_y = int(self.center_y - y * self.scale)  # Y축 반전
        return screen_x, screen_y
    
    def screen_to_world(self, screen_x, screen_y):
        """화면 좌표를 월드 좌표로 변환"""
        x = (screen_x - self.center_x) / self.scale
        y = (self.center_y - screen_y) / self.scale  # Y축 반전
        return x, y
    
    def generate_test_path(self, goal_x, goal_y, goal_yaw):
        """테스트용 경로 생성"""
        start = (0.0, 0.0, 0.0)
        goal = (goal_x, goal_y, goal_yaw)
        
        optimal_path = calc_optimal_path(*start, *goal, 1.0, STEP_SIZE)
        
        if optimal_path is None:
            print("경로 생성 실패!")
            return False
            
        self.path_points = list(zip(optimal_path.x, optimal_path.y, optimal_path.yaw))
        self.current_path_index = 0
        self.goal_position = goal
        print(f"경로 생성 완료: {len(self.path_points)}개 포인트")
        return True
    
    def find_lookahead_point(self):
        """Pure Pursuit 코드와 동일한 로직"""
        if not self.path_points:
            return None, -1
            
        # 동적 lookahead 거리 계산
        current_speed = abs(self.base_speed) * 0.1
        dynamic_lookahead = self.lookahead_gain * current_speed + self.lookahead_distance
        dynamic_lookahead = max(self.min_lookahead, min(dynamic_lookahead, self.max_lookahead))
        
        # 현재 진행 지점을 가상의 현재 위치로 설정
        if self.current_path_index >= len(self.path_points):
            self.current_path_index = len(self.path_points) - 1
            
        virtual_current = self.path_points[self.current_path_index]
        self.virtual_position = list(virtual_current)
        
        # Lookahead 포인트 찾기
        for i in range(self.current_path_index, len(self.path_points)):
            point = self.path_points[i]
            distance = math.sqrt(
                (point[0] - virtual_current[0])**2 + 
                (point[1] - virtual_current[1])**2
            )
            
            if distance >= dynamic_lookahead:
                return point, i
        
        return self.path_points[-1], len(self.path_points) - 1
    
    def calculate_pure_pursuit_steering(self, lookahead_point):
        """Pure Pursuit 조향각 계산"""
        virtual_current = self.virtual_position
        
        dx = lookahead_point[0] - virtual_current[0]
        dy = lookahead_point[1] - virtual_current[1]
        
        cos_yaw = math.cos(virtual_current[2])
        sin_yaw = math.sin(virtual_current[2])
        
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        
        lookahead_dist = math.sqrt(local_x**2 + local_y**2)
        
        if lookahead_dist < 0.01:
            return 0.0
            
        alpha = math.atan2(local_y, local_x)
        steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), lookahead_dist)
        
        return math.degrees(steering_angle)
    
    def update_simulation(self):
        """시뮬레이션 업데이트"""
        if not self.path_points or not self.auto_mode:
            return
            
        # Lookahead 점 찾기
        self.lookahead_point, _ = self.find_lookahead_point()
        
        if self.lookahead_point:
            # 조향각 계산
            self.steering_angle = self.calculate_pure_pursuit_steering(self.lookahead_point)
        
        # 경로 진행도 업데이트
        if self.current_path_index < len(self.path_points) - 1:
            speed_factor = abs(self.base_speed) / 100.0
            progress_increment = max(1, int(speed_factor * 1.5 * self.simulation_speed))  # 3 → 1.5로 반으로 줄임
            self.current_path_index = min(
                len(self.path_points) - 1,
                self.current_path_index + progress_increment
            )
    
    def is_goal_reached(self):
        """목표 도달 확인"""
        if not self.path_points or not self.goal_position:
            return False
            
        goal_point = self.path_points[-1]
        virtual_current = self.virtual_position
        
        distance_to_goal = math.sqrt(
            (goal_point[0] - virtual_current[0])**2 + 
            (goal_point[1] - virtual_current[1])**2
        )
        
        return distance_to_goal < self.goal_tolerance or self.current_path_index >= len(self.path_points) - 1
    
    def draw_path(self):
        """경로 그리기"""
        if len(self.path_points) < 2:
            return
            
        points = []
        for point in self.path_points:
            screen_pos = self.world_to_screen(point[0], point[1])
            points.append(screen_pos)
        
        if len(points) > 1:
            pygame.draw.lines(self.screen, self.GREEN, False, points, 3)
    
    def draw_car(self, pos, yaw, color=None):
        """차량 그리기"""
        if color is None:
            color = self.RED
            
        screen_pos = self.world_to_screen(pos[0], pos[1])
        
        # 차량 크기
        car_length = 20
        car_width = 12
        
        # 차량 사각형 그리기
        corners = [
            (-car_length//2, -car_width//2),
            (car_length//2, -car_width//2),
            (car_length//2, car_width//2),
            (-car_length//2, car_width//2)
        ]
        
        # 회전 적용
        rotated_corners = []
        for corner in corners:
            x = corner[0] * math.cos(yaw) - corner[1] * math.sin(yaw)
            y = corner[0] * math.sin(yaw) + corner[1] * math.cos(yaw)
            rotated_corners.append((screen_pos[0] + x, screen_pos[1] - y))
        
        pygame.draw.polygon(self.screen, color, rotated_corners)
        
        # 차량 방향 표시
        front_x = screen_pos[0] + car_length//2 * math.cos(yaw)
        front_y = screen_pos[1] - car_length//2 * math.sin(yaw)
        pygame.draw.circle(self.screen, self.WHITE, (int(front_x), int(front_y)), 3)
    
    def draw_lookahead(self):
        """Lookahead 포인트와 선 그리기"""
        if self.lookahead_point and self.virtual_position:
            # Lookahead 포인트
            screen_pos = self.world_to_screen(self.lookahead_point[0], self.lookahead_point[1])
            pygame.draw.circle(self.screen, self.YELLOW, screen_pos, 8)
            
            # 현재 위치에서 Lookahead 포인트로 선
            current_screen = self.world_to_screen(self.virtual_position[0], self.virtual_position[1])
            pygame.draw.line(self.screen, self.YELLOW, current_screen, screen_pos, 2)
    
    def draw_ui(self):
        """UI 정보 표시 - 제거됨"""
        pass  # 텍스트 모두 제거
    
    def draw_grid(self):
        """격자 그리기"""
        # 1미터 간격 격자 (범위 확장)
        for i in range(-6, 7):  # -6m ~ 6m으로 확장
            # 세로선
            start_pos = self.world_to_screen(i, -5)
            end_pos = self.world_to_screen(i, 5)
            pygame.draw.line(self.screen, self.GRAY, start_pos, end_pos, 1)
            
            # 가로선
            start_pos = self.world_to_screen(-6, i)
            end_pos = self.world_to_screen(6, i)
            pygame.draw.line(self.screen, self.GRAY, start_pos, end_pos, 1)
        
        # 원점 표시
        origin = self.world_to_screen(0, 0)
        pygame.draw.circle(self.screen, self.WHITE, origin, 5)
    
    def handle_events(self):
        """이벤트 처리"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # 왼쪽 클릭
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    world_x, world_y = self.screen_to_world(mouse_x, mouse_y)
                    
                    # 월드 좌표계 원점(0,0) 기준으로 yaw 설정
                    # 원점 위쪽(world_y > 0) → yaw = 90° (위쪽, ↑)
                    # 원점 아래쪽(world_y < 0) → yaw = -90° (아래쪽, ↓)  
                    # 원점 오른쪽(world_x > 0) → yaw = 0° (오른쪽, →)
                    # 원점 왼쪽(world_x < 0) → yaw = 180° (왼쪽, ←)
                    
                    if abs(world_y) > abs(world_x):  # Y축 방향이 더 큰 경우
                        if world_y > 0:
                            goal_yaw = math.pi/2      # 90° 위쪽 ↑
                        else:
                            goal_yaw = -math.pi/2     # -90° 아래쪽 ↓
                    else:  # X축 방향이 더 큰 경우
                        if world_x > 0:
                            goal_yaw = 0              # 0° 오른쪽 →
                        else:
                            goal_yaw = math.pi        # 180° 왼쪽 ←
                    
                    print(f"클릭 위치: ({world_x:.1f}, {world_y:.1f}), 설정 각도: {math.degrees(goal_yaw):.1f}°")
                    self.generate_test_path(world_x, world_y, goal_yaw)
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.auto_mode = not self.auto_mode
                elif event.key == pygame.K_r:
                    self.path_points = []
                    self.current_path_index = 0
                    self.goal_position = None
                elif event.key == pygame.K_EQUALS or event.key == pygame.K_PLUS:
                    self.simulation_speed = min(5, self.simulation_speed + 0.5)
                elif event.key == pygame.K_MINUS:
                    self.simulation_speed = max(0.1, self.simulation_speed - 0.5)
        
        return True
    
    def run(self):
        """메인 루프"""
        print("Pygame Pure Pursuit 테스트 시작!")
        print("마우스로 목표점을 클릭하세요.")
        
        # 초기 테스트 경로 생성
        self.generate_test_path(2.0, 1.5, math.pi/4)
        
        running = True
        while running:
            running = self.handle_events()
            
            # 시뮬레이션 업데이트
            self.update_simulation()
            
            # 화면 그리기
            self.screen.fill(self.BLACK)
            self.draw_grid()
            self.draw_path()
            
            if self.virtual_position:
                self.draw_car(self.virtual_position, self.virtual_position[2])
            
            if self.goal_position:
                # 목표 차량이 화면 범위 내에 있는지 확인
                goal_screen_x, goal_screen_y = self.world_to_screen(self.goal_position[0], self.goal_position[1])
                if 0 <= goal_screen_x <= 1000 and 0 <= goal_screen_y <= 800:
                    self.draw_car(self.goal_position, self.goal_position[2], self.BLUE)
                else:
                    # 화면 밖에 있으면 화살표나 표시로 방향 안내
                    font = pygame.font.Font(None, 24)
                    text = f"목표: ({self.goal_position[0]:.1f}, {self.goal_position[1]:.1f})"
                    text_surface = font.render(text, True, self.BLUE)
                    self.screen.blit(text_surface, (10, 200))
            
            self.draw_lookahead()
            self.draw_ui()
            
            pygame.display.flip()
            self.clock.tick(30)  # 30 FPS
        
        pygame.quit()

if __name__ == "__main__":
    test = PygamePurePursuitTest()
    test.run()