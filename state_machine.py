# state_machine.py
# CubeSat 도킹·충전 모듈 메인 상태 머신
#
# 상태 전이:
#   IDLE → APPROACH → SOFT_CAPTURE → HARD_LOCK → DEPLOY_SOLAR → CHARGING
#                                                              ↘ ERROR (각 단계)

import enum
import time
import config
from hardware import ToFSensorArray, Electromagnet, StepperMotor, SolarPanelArray, PogoPinCharger
from utils import get_logger

log = get_logger("StateMachine")


class State(enum.Enum):
    IDLE         = "IDLE"
    APPROACH     = "APPROACH"
    SOFT_CAPTURE = "SOFT_CAPTURE"
    HARD_LOCK    = "HARD_LOCK"
    DEPLOY_SOLAR = "DEPLOY_SOLAR"
    CHARGING     = "CHARGING"
    ERROR        = "ERROR"


class DockingStateMachine:

    def __init__(self):
        self.state = State.IDLE
        self._entry_time = 0.0
        self._error_msg  = ""

        # 하드웨어 인스턴스
        self.tof     = ToFSensorArray()
        self.magnet  = Electromagnet()
        self.stepper = StepperMotor()
        self.panels  = SolarPanelArray()
        self.pogo    = PogoPinCharger()

    def setup(self):
        """모든 하드웨어 초기화."""
        # TODO: 각 하드웨어 .setup() 호출
        pass

    def cleanup(self):
        """하드웨어 자원 해제 및 GPIO cleanup."""
        # TODO: 안전 정지 후 각 .cleanup() 호출
        pass

    def run(self, loop_hz: float = 10.0):
        """메인 루프. Ctrl+C로 종료."""
        # TODO: loop_hz 주기로 _tick() 반복 호출
        pass

    def step(self) -> State:
        """단일 틱 실행 (테스트용)."""
        self._tick()
        return self.state

    def reset(self):
        """ERROR → IDLE 수동 리셋."""
        # TODO: 액추에이터 정지 후 IDLE 전이
        pass

    # ── 상태 핸들러 ──────────────────────────────────────────

    def _tick(self):
        handlers = {
            State.IDLE:         self._handle_idle,
            State.APPROACH:     self._handle_approach,
            State.SOFT_CAPTURE: self._handle_soft_capture,
            State.HARD_LOCK:    self._handle_hard_lock,
            State.DEPLOY_SOLAR: self._handle_deploy_solar,
            State.CHARGING:     self._handle_charging,
            State.ERROR:        self._handle_error,
        }
        handlers[self.state]()

    def _handle_idle(self):
        # TODO: 전방 ToF 거리 < APPROACH_START_DIST_MM → APPROACH 전이
        pass

    def _handle_approach(self):
        # TODO: 스테퍼 전진 + 거리 감시
        # TODO: 타임아웃 체크
        # TODO: 거리 < SOFT_CAPTURE_DIST_MM → SOFT_CAPTURE 전이
        pass

    def _handle_soft_capture(self):
        # TODO: 전자석 ON
        # TODO: 거리 < HARD_LOCK_CONFIRM_MM → HARD_LOCK 전이
        # TODO: 타임아웃 체크
        pass

    def _handle_hard_lock(self):
        # TODO: Hard Lock 메커니즘 작동 (서보 or 솔레노이드)
        # TODO: 체결 확인 후 DEPLOY_SOLAR 전이
        pass

    def _handle_deploy_solar(self):
        # TODO: panels.deploy() 호출 → CHARGING 전이
        pass

    def _handle_charging(self):
        # TODO: pogo.is_contact_stable() 확인 → 충전 활성화
        # TODO: 접촉 끊김 → ERROR 전이
        pass

    def _handle_error(self):
        # TODO: 오류 메시지 로깅, 안전 유지
        pass

    # ── 헬퍼 ────────────────────────────────────────────────

    def _transition(self, new_state: State):
        log.info(f"{self.state.value} → {new_state.value}")
        self.state       = new_state
        self._entry_time = time.time()

    def _is_timeout(self, timeout_s: float) -> bool:
        return (time.time() - self._entry_time) > timeout_s

    def _error(self, msg: str):
        self._error_msg = msg
        log.error(f"오류: {msg}")
        self._transition(State.ERROR)

    def get_status(self) -> dict:
        """현재 상태 요약 반환 (모니터링용)."""
        # TODO: 센서·액추에이터 상태 포함한 dict 반환
        return {"state": self.state.value, "error": self._error_msg}
