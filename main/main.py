# main.py — 진입점: python3 main/main.py (cubeSat_docking/ 에서 실행)

import threading
import time
import sys, os
# 현재 파일(main/) 및 부모(cubeSat_docking/) 를 경로에 추가
_DIR    = os.path.dirname(os.path.abspath(__file__))
_PARENT = os.path.dirname(_DIR)
sys.path.insert(0, _PARENT)  # config 접근용
sys.path.insert(0, _DIR)     # main/ 내부 모듈 우선

from threads        import tof_thread, camera_thread, motor_thread
from state_machine  import state_machine_thread
from web_server     import app
from hardware       import GPIO_OK, _pwm, _servo_pwm, _cancel_approval
from hardware       import _set_magnet, _set_motor_target, _set_servo
from constants      import SERVO_STOWED_ANGLE, MOTOR_EXTENDED_STEPS
import shared

if __name__ == '__main__':
    # 시작 시 모터가 이미 6000 step 위치에 있다고 가정
    shared._motor_steps  = MOTOR_EXTENDED_STEPS
    shared._motor_target = MOTOR_EXTENDED_STEPS

    threading.Thread(target=tof_thread,           daemon=True, name="ToF").start()
    threading.Thread(target=camera_thread,        daemon=True, name="Camera").start()
    threading.Thread(target=motor_thread,         daemon=True, name="Motor").start()
    threading.Thread(target=state_machine_thread, daemon=True, name="StateMachine").start()

    print("=" * 52)
    print("  CubeSat Docking + Charging System  |  main_ver3.py (Semi-Auto)")
    print("  6-State: PRE → T.LOCK → S.CAP → H.LOCK → DOCKED → CHARGING")
    print("  반자동: 각 전이마다 웹 UI 승인 필요")
    print(f"  웹 UI : http://pi.local:5000")
    print("=" * 52)

    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        _running = False
        time.sleep(0.1)
        _cancel_approval()
        _set_magnet(False)
        _set_motor_target(0)
        _set_servo(SERVO_STOWED_ANGLE)
        if GPIO_OK:
            _pwm.stop()
            if _servo_pwm is not None:
                _servo_pwm.stop()
            G