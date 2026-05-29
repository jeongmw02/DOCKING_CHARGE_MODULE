# tests/test_stepper_manual.py
# 스테퍼 모터 수동 테스트 — 원하는 스텝까지 이동
#
# 핀 설정 (config.py 기준):
#   STEP_PIN = 12  (BCM)
#   DIR_PIN  = 17  (BCM)
#   EN_PIN   = 25  (BCM)
#
# 사용법:
#   python3 test_stepper_manual.py              # 기본값(3000 스텝 전진)
#   python3 test_stepper_manual.py 1000         # 1000 스텝 전진
#   python3 test_stepper_manual.py 1000 back    # 1000 스텝 후진
#   python3 test_stepper_manual.py 500 fwd 0.003  # 500 스텝 전진, 딜레이 3ms

import sys
import time
import RPi.GPIO as GPIO

# ── 핀 번호 (config.py와 동일) ─────────────────────────────
STEP_PIN = 12
DIR_PIN  = 17
EN_PIN   = 25

# ── 모터 파라미터 (config.py와 동일) ──────────────────────
STEPS_PER_REV = 200
MICROSTEP     = 8
LEAD_MM       = 8
STEPS_PER_MM  = (STEPS_PER_REV * MICROSTEP) / LEAD_MM  # 200 steps/mm

# ── 기본값 ─────────────────────────────────────────────────
DEFAULT_STEPS      = 3000
DEFAULT_DIRECTION  = 1      # 1=전진, 0=후진
DEFAULT_STEP_DELAY = 0.005  # 5ms (config.MOTOR_STEP_DELAY_S)


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(DIR_PIN,  GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(EN_PIN,   GPIO.OUT, initial=GPIO.HIGH)  # HIGH = disable
    print(f"[INIT] STEP={STEP_PIN}, DIR={DIR_PIN}, EN={EN_PIN}")


def enable():
    GPIO.output(EN_PIN, GPIO.LOW)


def disable():
    GPIO.output(EN_PIN, GPIO.HIGH)


def move_steps(steps: int, direction: int = 1, step_delay: float = DEFAULT_STEP_DELAY):
    """
    steps     : 이동할 스텝 수
    direction : 1=전진, 0=후진
    step_delay: 스텝 간 딜레이 (초). 작을수록 빠름.
    """
    dir_label = "전진(FWD)" if direction else "후진(BWD)"
    distance_mm = steps / STEPS_PER_MM
    print(f"[MOVE] {steps} steps ({distance_mm:.2f} mm) {dir_label} | delay={step_delay*1000:.1f}ms")

    GPIO.output(DIR_PIN, GPIO.HIGH if direction else GPIO.LOW)
    enable()

    try:
        for i in range(steps):
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(step_delay)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(step_delay)

            # 진행 상황 출력 (500 스텝마다)
            if (i + 1) % 500 == 0:
                print(f"  → {i+1}/{steps} steps ({(i+1)/STEPS_PER_MM:.2f} mm)")

    except KeyboardInterrupt:
        print("\n[STOP] 사용자 중단")
    finally:
        disable()

    print(f"[DONE] 완료: {steps} steps = {distance_mm:.2f} mm")


def cleanup():
    disable()
    GPIO.cleanup()
    print("[CLEANUP] GPIO 정리 완료")


if __name__ == "__main__":
    # 인자 파싱
    steps     = int(sys.argv[1])        if len(sys.argv) > 1 else DEFAULT_STEPS
    direction = 0 if (len(sys.argv) > 2 and sys.argv[2].lower() in ("back", "bwd", "0")) else 1
    delay     = float(sys.argv[3])      if len(sys.argv) > 3 else DEFAULT_STEP_DELAY

    print("=" * 50)
    print("  스테퍼 모터 수동 테스트")
    print(f"  목표: {steps} steps | {'전진' if direction else '후진'} | {delay*1000:.1f}ms/step")
    print("=" * 50)

    try:
        setup()
        input("Enter를 누르면 시작합니다...")
        move_steps(steps, direction, delay)
    finally:
        cleanup()
