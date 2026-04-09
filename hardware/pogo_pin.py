# hardware/pogo_pin.py
# Pogo Pin 충전 접점 감지 및 충전 회로 제어
#
# 감지 방식: POGO_DETECT_PIN 내부 풀업 입력
#   접촉 시 핀 → GND (LOW 감지)
# 충전 활성화: POGO_CHARGE_EN_PIN HIGH → 충전 IC 활성
#
# 의존 라이브러리: RPi.GPIO

import time
import config
from utils import get_logger

log = get_logger("PogoPins")


class ChargeStatus:
    def __init__(self, contact: bool, charging: bool, stable_s: float):
        self.contact_detected  = contact
        self.charging_enabled  = charging
        self.contact_stable_s  = stable_s


class PogoPinCharger:

    DEBOUNCE_MS      = 50
    STABLE_CONTACT_S = 0.5

    def __init__(self):
        self._charging      = False
        self._contact_since = None

    def setup(self):
        # TODO: GPIO.setup(DETECT_PIN, IN, PUD_UP)
        # TODO: GPIO.setup(CHARGE_EN_PIN, OUT, LOW)
        # TODO: GPIO.add_event_detect(DETECT_PIN, BOTH, callback=_on_change, bouncetime=50)
        log.info("Pogo pin 초기화 완료")

    def is_contact_detected(self) -> bool:
        # TODO: return GPIO.input(POGO_DETECT_PIN) == LOW
        return False

    def is_contact_stable(self) -> bool:
        """STABLE_CONTACT_S 이상 접촉 유지 여부 확인."""
        # TODO: 접촉 중이면 _contact_since 기록 후 경과 시간 체크
        return False

    def enable_charging(self) -> bool:
        """안정 접촉 확인 후 충전 활성화."""
        # TODO: is_contact_stable() 검증
        # TODO: GPIO.output(CHARGE_EN_PIN, HIGH)
        self._charging = True
        log.info("충전 활성화")
        return True

    def disable_charging(self):
        # TODO: GPIO.output(CHARGE_EN_PIN, LOW)
        self._charging = False
        log.info("충전 비활성화")

    def get_status(self) -> ChargeStatus:
        contact = self.is_contact_detected()
        stable_s = (time.time() - self._contact_since) if self._contact_since and contact else 0.0
        return ChargeStatus(contact, self._charging, stable_s)

    def cleanup(self):
        self.disable_charging()
        # TODO: GPIO.remove_event_detect(DETECT_PIN)
        log.info("Pogo pin 정리 완료")

    def _on_contact_change(self, channel: int):
        """GPIO 인터럽트 콜백."""
        # TODO: 접촉 해제 시 충전 즉시 중단
        pass
