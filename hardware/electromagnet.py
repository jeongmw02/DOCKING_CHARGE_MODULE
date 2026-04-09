# hardware/electromagnet.py
# 전자석 PWM 제어 드라이버
#
# 제어 전략:
#   흡착 → PULL_DUTY(100%) × PULL_MS → HOLD_DUTY(80%) 유지 (과열 방지)
#   해제 → 즉시 0%
#
# 배선: GPIO → MOSFET Gate → 전자석 코일
# 주의: 코일 양단에 프리휠링 다이오드(flyback diode) 필수!

import config
from utils import get_logger

log = get_logger("Electromagnet")


class Electromagnet:

    def __init__(self):
        self._pwm      = None
        self._engaged  = False

    def setup(self):
        # TODO: GPIO.setup(ELECTROMAGNET_PIN, OUT)
        # TODO: GPIO.PWM(pin, 1000Hz).start(0)
        log.info(f"전자석 초기화 (GPIO{config.ELECTROMAGNET_PIN})")

    def engage(self):
        """흡착: Pull → Hold 자동 전환."""
        # TODO: PWM duty = PULL_DUTY
        # TODO: threading.Timer(PULL_MS/1000, → duty = HOLD_DUTY)
        self._engaged = True
        log.info("전자석 ON")

    def release(self):
        """즉시 해제."""
        # TODO: PWM duty = 0
        self._engaged = False
        log.info("전자석 OFF")

    @property
    def is_engaged(self) -> bool:
        return self._engaged

    def cleanup(self):
        self.release()
        # TODO: self._pwm.stop()
        log.info("전자석 정리 완료")
