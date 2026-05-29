# tests/test_stepper_calibrate.py
# 스테퍼 모터 기준점(캘리브레이션) 설정
#
# 흐름:
#   1. 방향 선택 (up / down)
#   2. 모터가 해당 방향으로 계속 이동
#   3. Enter → 즉시 정지 → 기준점 1 저장
#   4. 반대 방향으로 자동 재출발
#   5. Enter → 즉시 정지 → 기준점 2 저장
#   6. 결과 출력 (steps, mm, 총 이동 범위)
#
# 핀 설정 (config.py 기준):
#   STEP_PIN = 12, DIR_PIN = 17, EN_PIN = 25

import sys
import os
import time
import threading
import json

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import RPi.GPIO as GPIO

# ── 핀 / 파라미터 ──────────────────────────────────────────
STEP_PIN      = 12
DIR_PIN       = 17
EN_PIN        = 25
STEPS_PER_REV = 200
MICROSTEP     = 8
LEAD_MM       = 8
STEPS_PER_MM  = (STEPS_PER_REV * MICROSTEP) / LEAD_MM   # 200 steps/mm
STEP_DELAY    = 0.0025   # 5ms/step (느리게 → 기준점 정밀도 향상)

CALIB_FILE    = os.path.join(os.path.dirname(__file__), '..', 'calibration.json')


# ── GPIO ───────────────────────────────────────────────────
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(DIR_PIN,  GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(EN_PIN,   GPIO.OUT, initial=GPIO.HIGH)

def enable():
    GPIO.output(EN_PIN, GPIO.LOW)

def disable():
    GPIO.output(EN_PIN, GPIO.HIGH)

def cleanup():
    disable()
    GPIO.cleanup()


# ── 이동 스레드 ────────────────────────────────────────────
class MotorRunner:
    """백그라운드에서 모터를 계속 돌리다가 stop() 시 즉시 정지."""

    def __init__(self):
        self._stop_event = threading.Event()
        self._thread     = None
        self.steps_moved = 0      # 이번 구간 누적 스텝

    def start(self, direction: int):
        """direction: 1=up(전진), 0=down(후진)"""
        self._stop_event.clear()
        self.steps_moved = 0
        GPIO.output(DIR_PIN, GPIO.HIGH if direction else GPIO.LOW)
        enable()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join()
        disable()

    def _run(self):
        while not self._stop_event.is_set():
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEP_DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEP_DELAY)
            self.steps_moved += 1


# ── 캘리브레이션 ───────────────────────────────────────────
def wait_for_stop(runner: MotorRunner, label: str) -> int:
    """Enter 입력까지 모터 이동, 정지 후 누적 스텝 반환."""
    print(f"\n  ▶ 이동 중... [Enter]를 누르면 '{label}' 기준점으로 설정")
    input()          # 블로킹 — Enter 대기
    runner.stop()
    steps = runner.steps_moved
    mm    = steps / STEPS_PER_MM
    print(f"  ✔ {label} 설정 | {steps} steps ({mm:.2f} mm)")
    return steps


def save_calibration(ref1_steps: int, ref2_steps: int, direction: str):
    total_steps = ref1_steps + ref2_steps
    data = {
        "direction_first": direction,
        "ref1_steps": ref1_steps,
        "ref1_mm":    round(ref1_steps / STEPS_PER_MM, 3),
        "ref2_steps": ref2_steps,
        "ref2_mm":    round(ref2_steps / STEPS_PER_MM, 3),
        "total_range_steps": total_steps,
        "total_range_mm":    round(total_steps / STEPS_PER_MM, 3),
        "steps_per_mm": STEPS_PER_MM,
    }
    with open(CALIB_FILE, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    return data


def run_calibration(first_direction: str):
    dir1 = 1 if first_direction == 'up' else 0
    dir2 = 1 - dir1                          # 반대 방향
    dir1_label = first_direction.upper()
    dir2_label = ('down' if first_direction == 'up' else 'up').upper()

    runner = MotorRunner()

    print(f"\n[1단계] {dir1_label} 방향으로 이동 시작")
    runner.start(dir1)
    ref1 = wait_for_stop(runner, "기준점 1")

    time.sleep(0.3)

    print(f"\n[2단계] {dir2_label} 방향(반대)으로 이동 시작")
    runner.start(dir2)
    ref2 = wait_for_stop(runner, "기준점 2")

    return ref1, ref2


# ── 메인 ──────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 50)
    print("  스테퍼 모터 기준점 캘리브레이션")
    print("=" * 50)

    # 방향 선택
    if len(sys.argv) > 1 and sys.argv[1].lower() in ('up', 'down'):
        first_dir = sys.argv[1].lower()
    else:
        while True:
            raw = input("\n첫 번째 이동 방향을 입력하세요 (up / down): ").strip().lower()
            if raw in ('up', 'down'):
                first_dir = raw
                break
            print("  'up' 또는 'down'으로 입력하세요.")

    print(f"\n방향: {first_dir.upper()} → 정지 → {'DOWN' if first_dir=='up' else 'UP'}")
    input("준비되면 Enter를 눌러 시작...")

    setup()

    try:
        ref1, ref2 = run_calibration(first_dir)
        data = save_calibration(ref1, ref2, first_dir)

        print("\n" + "=" * 50)
        print("  캘리브레이션 완료")
        print(f"  기준점 1 : {data['ref1_steps']} steps = {data['ref1_mm']} mm")
        print(f"  기준점 2 : {data['ref2_steps']} steps = {data['ref2_mm']} mm")
        print(f"  전체 범위: {data['total_range_steps']} steps = {data['total_range_mm']} mm")
        print(f"  저장 위치: calibration.json")
        print("=" * 50)

    except KeyboardInterrupt:
        print("\n[중단] 캘리브레이션 취소")
    finally:
        cleanup()
