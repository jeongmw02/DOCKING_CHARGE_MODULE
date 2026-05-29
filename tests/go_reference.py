# tests/test_stepper_go_ref1.py
# calibration.json의 기준점 1로 복귀
#
# 캘리브레이션 흐름 기준:
#   기준점 1: first_direction 방향으로 이동 후 정지한 위치
#   기준점 2: 반대 방향으로 이동 후 정지한 위치  ← 현재 위치 가정
#
#   → 기준점 1로 돌아가려면 first_direction 방향으로 ref1_steps만큼 이동
#
# 사용법:
#   python3 test_stepper_go_ref1.py          # calibration.json 자동 로드
#   python3 test_stepper_go_ref1.py 0.003    # 딜레이(초) 직접 지정

import sys
import os
import time
import json

import RPi.GPIO as GPIO

# ── 핀 설정 ───────────────────────────────────────────────
STEP_PIN      = 12
DIR_PIN       = 17
EN_PIN        = 25
STEP_DELAY    = float(sys.argv[1]) if len(sys.argv) > 1 else 0.005

CALIB_FILE = os.path.join(os.path.dirname(__file__), '..', 'calibration.json')


def load_calibration():
    if not os.path.exists(CALIB_FILE):
        print("[ERROR] calibration.json 없음. 먼저 test_stepper_calibrate.py 실행하세요.")
        sys.exit(1)
    with open(CALIB_FILE) as f:
        return json.load(f)


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(DIR_PIN,  GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(EN_PIN,   GPIO.OUT, initial=GPIO.HIGH)


def move_steps(steps: int, direction: int, step_delay: float):
    GPIO.output(DIR_PIN, GPIO.HIGH if direction else GPIO.LOW)
    GPIO.output(EN_PIN, GPIO.LOW)   # enable

    try:
        for i in range(steps):
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(step_delay)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(step_delay)
            if (i + 1) % 500 == 0:
                print(f"  → {i+1}/{steps} steps")
    except KeyboardInterrupt:
        print("\n[STOP] 중단")
    finally:
        GPIO.output(EN_PIN, GPIO.HIGH)  # disable


def cleanup():
    GPIO.output(EN_PIN, GPIO.HIGH)
    GPIO.cleanup()


if __name__ == "__main__":
    calib = load_calibration()

    steps     = calib['ref1_steps']
    mm        = calib['ref1_mm']
    # 기준점 1은 first_direction으로 이동한 위치이므로
    # 현재(기준점 2)에서 돌아가려면 first_direction 방향으로 이동
    dir_str   = calib['direction_first']
    direction = 1 if dir_str == 'up' else 0

    print("=" * 50)
    print("  기준점 1 복귀")
    print(f"  방향 : {dir_str.upper()}")
    print(f"  거리 : {steps} steps ({mm} mm)")
    print(f"  딜레이: {STEP_DELAY*1000:.1f} ms/step")
    print("=" * 50)
    input("Enter를 누르면 시작합니다...")

    setup()
    try:
        move_steps(steps, direction, STEP_DELAY)
        print(f"\n[DONE] 기준점 1 도착 ({mm} mm)")
    finally:
        cleanup()
