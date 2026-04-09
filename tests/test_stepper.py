# tests/test_stepper.py
# 스테퍼 모터 이동 확인 스크립트
# 실행: python tests/test_stepper.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hardware import StepperMotor
import time

def test_stepper():
    motor = StepperMotor()
    motor.setup()

    print("전진 50mm")
    motor.move_mm(50, direction=StepperMotor.DIR_FORWARD, speed_mm_s=10, blocking=True)
    print(f"위치: {motor.position_mm:.1f} mm")
    time.sleep(1)

    print("후진 50mm")
    motor.move_mm(50, direction=StepperMotor.DIR_BACKWARD, speed_mm_s=10, blocking=True)
    print(f"위치: {motor.position_mm:.1f} mm")

    motor.cleanup()
    print("테스트 완료")

if __name__ == "__main__":
    test_stepper()
