# hardware/stepper.py
# NEMA17 스테퍼 모터 드라이버 (DRV8825 / A4988 호환)
#
# 인터페이스: STEP / DIR / EN 핀 (STEP/DIR 방식)
# 리드스크류 변환:
#   steps_per_mm = (STEPS_PER_REV × MICROSTEP) / LEAD_MM
#
# 의존 라이브러리: RPi.GPIO

import threading
import config
from utils import get_logger

log = get_logger("Stepper")


class StepperMotor:

    DIR_FORWARD  = 1
    DIR_BACKWARD = 0

    def __init__(self):
        self.steps_per_mm  = (config.STEPPER_STEPS_PER_REV * config.STEPPER_MICROSTEP) / config.STEPPER_LEAD_MM
        self._position_mm  = 0.0
        self._stop_flag    = threading.Event()
        self._move_thread  = None

    def setup(self):
        # TODO: GPIO.setup(STEP_PIN, OUT)
        # TODO: GPIO.setup(DIR_PIN, OUT)
        # TODO: GPIO.setup(EN_PIN, OUT, initial=HIGH)  ← disable
        log.info(f"스테퍼 초기화 | {self.steps_per_mm:.1f} steps/mm")

    def move_mm(self, distance_mm: float, direction: int = DIR_FORWARD,
                speed_mm_s: float = None, blocking: bool = True):
        """지정 거리(mm) 이동. blocking=False면 백그라운드 실행."""
        # TODO: steps = int(distance_mm * steps_per_mm)
        # TODO: step_delay = 1 / (speed * steps_per_mm * 2)
        # TODO: blocking → _do_steps() 직접 호출
        # TODO: non-blocking → Thread(_do_steps) 실행
        log.info(f"이동 명령: {distance_mm}mm {'전진' if direction else '후진'}")

    def stop(self):
        """진행 중 이동 즉시 중단."""
        # TODO: _stop_flag.set() → thread join
        log.info("스테퍼 정지")

    def is_moving(self) -> bool:
        return self._move_thread is not None and self._move_thread.is_alive()

    @property
    def position_mm(self) -> float:
        return self._position_mm

    def cleanup(self):
        self.stop()
        log.info("스테퍼 정리 완료")

    def _enable(self):
        pass  # TODO: GPIO.output(EN_PIN, LOW)

    def _disable(self):
        pass  # TODO: GPIO.output(EN_PIN, HIGH)

    def _do_steps(self, steps: int, direction: int, step_delay: float):
        """실제 STEP 펄스 생성 루프."""
        # TODO: _enable()
        # TODO: GPIO.output(DIR_PIN, direction)
        # TODO: for i in range(steps): STEP HIGH → sleep → LOW → sleep
        #         if _stop_flag: break
        #         _position_mm 누적
        # TODO: _disable()
        pass
