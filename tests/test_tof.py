# tests/test_tof.py
# VL53L0X ToF 센서 단독 동작 확인 스크립트
# 실행: python tests/test_tof.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hardware.tof_sensor import ToFSensor
import time

def test_tof():
    tof = ToFSensor()
    tof.setup()

    print("ToF 센서 측정 시작 (Ctrl+C 종료)")
    print("-" * 30)
    try:
        while True:
            r = tof.read()
            status = "OK" if r.valid else "범위초과"
            print(f"  거리: {r.distance_mm:6d} mm  [{status}]")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        tof.cleanup()

if __name__ == "__main__":
    test_tof()
