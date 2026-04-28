# hardware/electromagnet.py
# 전자석 PWM 제어 드라이버
#
# 제어 전략:
#   흡착 → PULL_DUTY(100%) × PULL_MS → HOLD_DUTY(40%) 유지 (과열 방지)
#   해제 → 즉시 0%
#
# 배선: GPIO18 → XY-MOS TRIG/PWM → 전자석 코일
# 주의: 코일 양단에 프리휠링 다이오드(1N4007) 필수!

import time
import threading
import RPi.GPIO as GPIO
import config
from utils import get_logger

log = get_logger("Electromagnet")

EM_PWM_FREQ = 1000  # Hz


class Electromagnet:

    def __init__(self):
        self._pwm     = None
        self._engaged = False
        self._lock    = threading.Lock()

    def setup(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(config.ELECTROMAGNET_PIN, GPIO.OUT, initial=GPIO.LOW)
        self._pwm = GPIO.PWM(config.ELECTROMAGNET_PIN, EM_PWM_FREQ)
        self._pwm.start(0)
        log.info(f"전자석 초기화 완료 (GPIO{config.ELECTROMAGNET_PIN}, {EM_PWM_FREQ}Hz)")

    def engage(self):
        """흡착: Pull(100%) → Hold(40%) 자동 전환."""
        with self._lock:
            if self._engaged:
                log.debug("전자석 이미 흡착 중")
                return
            log.info(f"전자석 흡착 시작 (Pull {config.ELECTROMAGNET_PULL_DUTY}%)")
            self._pwm.ChangeDutyCycle(config.ELECTROMAGNET_PULL_DUTY)
            time.sleep(config.ELECTROMAGNET_PULL_MS / 1000.0)
            self._pwm.ChangeDutyCycle(config.ELECTROMAGNET_HOLD_DUTY)
            self._engaged = True
            log.info(f"전자석 유지 모드 (Hold {config.ELECTROMAGNET_HOLD_DUTY}%)")

    def release(self):
        """즉시 해제."""
        with self._lock:
            if self._pwm:
                self._pwm.ChangeDutyCycle(0)
            self._engaged = False
            log.info("전자석 OFF")

    @property
    def is_engaged(self) -> bool:
        return self._engaged

    def cleanup(self):
        self.release()
        if self._pwm:
            self._pwm.stop()
        GPIO.cleanup(config.ELECTROMAGNET_PIN)
        log.info("전자석 정리 완료")
