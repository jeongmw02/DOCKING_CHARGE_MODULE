# tests/test_state_machine.py
# 상태 머신 전이 흐름 확인 스크립트 (하드웨어 없이 수동 시뮬레이션)
# 실행: python tests/test_state_machine.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from state_machine import DockingStateMachine, State

def test_state_transitions():
    sm = DockingStateMachine()
    sm.setup()

    print(f"초기 상태: {sm.state.value}")
    assert sm.state == State.IDLE

    # TODO: 각 상태 전이를 수동으로 트리거해서 검증
    # 예시:
    #   sm.tof.mock_distance(250)   → APPROACH 전이 확인
    #   sm.tof.mock_distance(40)    → SOFT_CAPTURE 전이 확인
    #   ...

    print("상태 머신 시뮬레이션 테스트 완료")
    sm.cleanup()

if __name__ == "__main__":
    test_state_transitions()
