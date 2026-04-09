# tests/test_tof.py
# ToF 센서 단독 동작 확인 스크립트
# 실행: python tests/test_tof.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hardware import ToFSensorArray
import time

def test_tof_readings():
    tof = ToFSensorArray()
    tof.setup()

    print("ToF 센서 측정 시작 (Ctrl+C 종료)")
    try:
        while True:
            readings = tof.read_all()
            for r in readings:
                status = "OK" if r.valid else "INVALID"
                print(f"  ToF[{r.index}] = {r.distance_mm:6.1f} mm [{status}]")
            print(f"  정렬 오차 = {tof.get_alignment_offset_mm():.1f} mm")
            print("---")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        tof.cleanup()

if __name__ == "__main__":
    test_tof_readings()
