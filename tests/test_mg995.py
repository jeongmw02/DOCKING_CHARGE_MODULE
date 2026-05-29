# tests/test_mg995r_angle.py
# MG995R 각도 제어 (시간 기반) — GPIO 13번
#
# MG995R은 위치 피드백이 없어서 "속도 × 시간 = 각도"로 추정.
# 기본값: 절반 속도(50%) 사용 → 안정적, 반복성 좋음
#
# ※ 실제 모터마다 오차 있음 → DEG_PER_SEC 값으로 보정 가능
#
# 조작:
#   +  또는 숫자 입력 → +10도
#   -                → -10도
#   숫자 직접 입력    → 해당 각도만큼 회전 (예: 30, -45)
#   r                → 현재 누적 각도 출력
#   q                → 종료

import sys
import time
import RPi.GPIO as GPIO

# ── 핀 설정 ───────────────────────────────────────────────
SIGNAL_PIN = 13
FREQ_HZ    = 50
PERIOD_MS  = 20.0

# ── 펄스 범위 ──────────────────────────────────────────────
STOP_PULSE = 1.5   # ms
MIN_PULSE  = 0.5   # ms (+방향 최대)
MAX_PULSE  = 2.5   # ms (-방향 최대)

# ── 속도/각도 보정 ─────────────────────────────────────────
# MG995R @ 50% 속도 기준 실측값. 실제와 다르면 이 값을 조정하세요.
# 예: 실제로 1초에 150도 돌면 DEG_PER_SEC = 150
DRIVE_SPEED  = 50        # % (고정 구동 속도)
DEG_PER_SEC  = 160.0     # 50% 속도에서 초당 회전 각도 (보정 필요시 수정)
STEP_DEG     = 10        # 기본 증감 단위 (도)


def speed_to_duty(speed: float) -> float:
    speed = max(-100.0, min(100.0, speed))
    pulse_ms = STOP_PULSE - (speed / 100.0) * (STOP_PULSE - MIN_PULSE)
    return (pulse_ms / PERIOD_MS) * 100.0


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(SIGNAL_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SIGNAL_PIN, FREQ_HZ)
    pwm.start(speed_to_duty(0))
    print(f"[INIT] GPIO {SIGNAL_PIN} | MG995R | 구동속도 {DRIVE_SPEED}% | {DEG_PER_SEC}°/s")
    return pwm


def rotate_deg(pwm, degrees: float):
    """degrees 만큼 회전. 양수=정방향, 음수=역방향."""
    if degrees == 0:
        return

    direction = 1 if degrees > 0 else -1
    duration  = abs(degrees) / DEG_PER_SEC
    speed     = DRIVE_SPEED * direction

    print(f"  → {'정방향' if direction > 0 else '역방향'} {abs(degrees):.1f}° ({duration:.3f}s)")
    pwm.ChangeDutyCycle(speed_to_duty(speed))
    time.sleep(duration)
    pwm.ChangeDutyCycle(speed_to_duty(0))
    time.sleep(0.1)   # 관성 안정


def cleanup(pwm):
    pwm.ChangeDutyCycle(speed_to_duty(0))
    time.sleep(0.2)
    pwm.stop()
    GPIO.cleanup()
    print("[CLEANUP] 완료")


if __name__ == "__main__":
    pwm = setup()
    total_deg = 0.0

    print(f"\n조작법:")
    print(f"  +        → +{STEP_DEG}도")
    print(f"  -        → -{STEP_DEG}도")
    print(f"  숫자     → 해당 각도 회전 (예: 30, -45, 90)")
    print(f"  r        → 누적 각도 확인")
    print(f"  q        → 종료\n")

    try:
        while True:
            raw = input("입력: ").strip()

            if raw.lower() == 'q':
                break

            elif raw.lower() == 'r':
                print(f"  누적 각도: {total_deg:.1f}°")
                continue

            elif raw == '+':
                deg = STEP_DEG

            elif raw == '-':
                deg = -STEP_DEG

            else:
                try:
                    deg = float(raw)
                except ValueError:
                    print("  '+' / '-' / 숫자 / 'r' / 'q' 로 입력하세요.")
                    continue

            rotate_deg(pwm, deg)
            total_deg += deg
            print(f"  누적: {total_deg:.1f}°")

    except KeyboardInterrupt:
        print("\n[STOP]")
    finally:
        cleanup(pwm)
