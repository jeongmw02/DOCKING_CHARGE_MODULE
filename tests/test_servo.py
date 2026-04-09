# tests/test_servo.py
# 서보 + 솔라 패널 전개/수납 확인 스크립트
# 실행: python tests/test_servo.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hardware import SolarPanelArray
import time

def test_solar_panels():
    panels = SolarPanelArray()
    panels.setup()

    print("솔라 패널 전개")
    panels.deploy()
    time.sleep(2)

    print("솔라 패널 수납")
    panels.stow()
    time.sleep(1)

    panels.cleanup()
    print("테스트 완료")

if __name__ == "__main__":
    test_solar_panels()
