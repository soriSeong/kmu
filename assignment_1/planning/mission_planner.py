#!/usr/bin/env python3
import time
import threading

class MissionPlanner(threading.Thread):
    """
    Mission Planning) 클래스:
    부모 노드의 shared 데이터를 통해 Perception, Plan 객체를 참조해 주기마다 새로운 미션을 결정한다.
    ros와 별도 처리를 위해 스레드를 사용했다. Node의 메인 루프와 별도로 실행된다.
    """
    def __init__(self, parent, rate):
        """
        parent: 상위 노드 객체, shared 속성을 통해 데이터 접근한다.
        rate: 실행 주기
        super().__init__()를 통해 Thread 초기화한다.
        period 계산(1.0 / rate)하여 루프 대기 시간 설정한다.
        Plan 객체(parent.plan)와 Perception 객체(parent.shared.perception) 참조 저장한다.
        """
        super().__init__()
        self.period = 1.0 / rate
        self.plan = parent.plan
        self.perception = parent.shared.perception
        
    def run(self):
        """
        스레드 진입점
        무한 루프를 돌며 Perception 정보를 기반으로 mission_decision, behavior_decision, motion_decision을 갱신한다.
        
        동작은 크게 4가지로 나뉜다, 신호등, 라바콘 회피, 차량 회피, 일반 차선 인식 주행
        1. traffic_light 상태가 False일 경우(아직 신호등 미션을 끝내지 않음)
        2. cone_list에 라바콘(작은 장애물)이 감지된 경우
        3. vehicle_list에 차량(큰 장애물)이 감지된 경우
        4. 위 세 가지 상황 모두 해당되지 않으면 차선 인식 주행을 실행한다.
        
        예외가 발생하면 에러 메시지 출력한다.
        주기(self.period)만큼 time.sleep하여 다음 루프 대기한다.
        """
        while True:
            try:
                # 신호등 미션
                if self.perception.traffic_light == False:
                    self.plan.mission_decision = "traffic_light"
                    self.plan.behavior_decision = "traffic_light"
                    self.plan.motion_decision = "traffic_light"

                # 라바콘 회피 미션
                elif len(self.perception.cone_list) > 0:
                    self.plan.mission_decision = "obs_small"
                    self.plan.behavior_decision = "obs_small"
                    self.plan.motion_decision = "obs_small"

                # 차량 회피 미션
                elif len(self.perception.vehicle_list) > 0:
                    self.plan.mission_decision = "obs_big"
                    self.plan.behavior_decision = "obs_big"
                    self.plan.motion_decision = "obs_big"

                # 장애물이 없으면 일반 주행 모드(차선 인식 주행)로 설정
                else:
                    self.plan.mission_decision = "go"
                    self.plan.behavior_decision = "go"
                    self.plan.motion_decision = "go"

            except Exception as e:
                print(f"Mission Planner Exception: {e}")
                import traceback
                traceback.print_exc()

            time.sleep(self.period)
