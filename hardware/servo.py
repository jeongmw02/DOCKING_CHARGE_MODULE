# hardware/servo.py
# 서보 모터 PWM 제어 드라이버 (솔라 패널 전개)
#
# 50Hz 표준 서보 듀티 변환:
#   duty(%) = ((min_ms + (max_ms - min_ms) × angle/180) / 20ms) × 100
#
# 의존 라이브러리: RPi.GPIO

import time
import config
from utils import get_logger

log = get_logger("Servo")


class ServoMotor:

    def __init__(self, pin: int, name: str = "Servo"):
        self._pin   = pin
        self._name  = name
        self._pwm   = None
        self._angle = None

    def setup(self):
        # TODO: GPIO.setup(pin, OUT)
        # TODO: GPIO.PWM(pin, SERVO_FREQ_HZ).start(0)
        log.info(f"[{self._name}] 서보 초기화 (GPIO{self._pin})")

    def move_to(self, angle_deg: float, delay_s: float = 0.5):
        """목표 각도로 이동 후 신호 차단 (발열 방지)."""
        # TODO: duty = _angle_to_duty(angle_deg)
        # TODO: pwm.ChangeDutyCycle(duty) → sleep(delay_s) → duty(0)
        self._angle = angle_deg
        log.info(f"[{self._name}] → {angle_deg}°")

    def deploy(self):
        self.move_to(config.SOLAR_PANEL_DEPLOYED_DEG)

    def stow(self):
        self.move_to(config.SOLAR_PANEL_STOWED_DEG)

    @property
    def angle(self):
        return self._angle

    def cleanup(self):
        # TODO: pwm.stop()
        log.info(f"[{self._name}] 서보 정리 완료")

    def _angle_to_duty(self, angle_deg: float) -> float:
        # TODO: 공식 적용 → duty 반환
        return 0.0


class SolarPanelArray:
    """두 서보를 묶어 대칭 전개하는 헬퍼."""

    def __init__(self):
        self._a = ServoMotor(config.SERVO_PANEL_A_PIN, "PanelA")
        self._b = ServoMotor(config.SERVO_PANEL_B_PIN, "PanelB")

    def setup(self):
        self._a.setup()
        self._b.setup()

    def deploy(self):
        # TODO: 순차 전개 (전류 피크 분산)
        self._a.deploy()
        time.sleep(0.1)
        self._b.deploy()
        log.info("솔라 패널 전개 완료")

    def stow(self):
        self._b.stow()
        time.sleep(0.1)
        self._a.stow()

    def cleanup(self):
        self._a.cleanup()
        self._b.cleanup()
