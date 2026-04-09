# tests/test_pogo.py
# Pogo pin 접촉 감지 및 충전 활성화 확인 스크립트
# 실행: python tests/test_pogo.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hardware import PogoPinCharger
import time

def test_pogo():
    pogo = PogoPinCharger()
    pogo.setup()

    print("Pogo pin 상태 모니터링 (Ctrl+C 종료)")
    try:
        while True:
            status = pogo.get_status()
            print(
                f"  접촉: {status.contact_detected} | "
                f"안정: {status.contact_stable_s:.1f}s | "
                f"충전: {status.charging_enabled}"
            )
            if status.contact_detected and not status.charging_enabled:
                pogo.enable_charging()
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        pogo.cleanup()

if __name__ == "__main__":
    test_pogo()
