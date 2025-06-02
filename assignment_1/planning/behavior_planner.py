#!/usr/bin/env python3
import threading
from time import sleep
from .sub_function.behavior import Behavior

class BehaviorPlanner(threading.Thread):
    """
    Behavior Planning 클래스:
    부모 노드의 shared 데이터를 가져와 독립적으로 주기마다 행동 결정을 처리한다.
    ros와 별도 처리를 위해 스레드를 사용했다. Node의 메인 루프와 별도로 실행된다.
    """
    def __init__(self, parent, rate):
        """
        parent: 상위 노드(또는 Main) 객체, 여기서 shared 속성을 사용하여 전역 데이터 접근
        rate: 실행 주기(Hz)
        Thread 초기화(super().__init__())
        period 계산: 1.0 / rate (루프 대기 시간)
        Shared, Perception, Plan 객체 참조 저장
        Behavior 모듈 인스턴스 생성:
        pc(self.perception): Perception 객체 전달
        pl(self.plan): Plan 객체 전달
        """
        super().__init__()
        self.period = 1.0 / rate
        self.shared = parent.shared
        self.perception = self.shared.perception
        self.plan = self.shared.plan
        self.behavior = Behavior(
            pc=self.perception,
            pl=self.plan
        )
    
    def run(self):
        """
        스레드 진입점(run)
        무한 루프를 돌며 plan.behavior_decision 값을 확인하고,
        해당 행동 메서드를 호출한다.
        크게 미션 별 행동 4가지로 나뉘는데,
        1. plan.behavior_decision이 "go"이면 behavior.go()를 호출한다.
        2. plan.behavior_decision이 "traffic_light"이면 behavior.traffic_light_decision()를 호출한다.
        3. plan.behavior_decision이 "obs_small"이면 behavior.obs_small()를 호출한다.
        4. plan.behavior_decision이 "obs_big"이면 behavior.obs_big()를 호출한다.
        
        그 외(IndexError 등) 예외 발생 시 에러 메시지 출력한다.

        주기만큼 sleep한다.
        """
        while True:
            try:
                if self.plan.behavior_decision == "go":
                    self.behavior.go()
                
                elif self.plan.behavior_decision == "traffic_light":
                    self.behavior.traffic_light_decision()
                # 라바콘
                elif self.plan.behavior_decision == "obs_small":
                    self.behavior.obs_small()
                # 차량 회피
                elif self.plan.behavior_decision == "obs_big":
                    self.behavior.obs_big()

            except IndexError:
                print("++++++++behavior_planner+++++++++")

            sleep(self.period)