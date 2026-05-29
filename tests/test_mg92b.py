# tests/test_mg92b_manual.py
# MG92B 서보 모터 수동 테스트 — GPIO 13번, 원하는 각도로 이동
#
# 핀 설정:
#   SIGNAL PIN = GPIO 13 (BCM)
#   VCC        = 5V
#   GND        = GND
#
# 사용법:
#   python3 test_mg92b_manual.py            # 대화형 모드 (각도 직접 입력)
#   python3 test_mg92b_manual.py 90         # 90도로 이동 후 종료
#   python3 test_mg92b_manual.py 0 90 180   # 0→90→180도 순서대로 이동
#
# MG92B 스펙:
#   동작 범위: 0° ~ 180°
#   펄스폭: 0.5ms (0°) ~ 2.5ms (180°)  @ 50Hz

import sys
import time
import RPi.GPIO as GPIO

# ── 핀 설정 ───────────────────────────────────────────────
SIGNAL_PIN  = 13      # BCM GPIO 13
FREQ_HZ     = 50      # 서보 PWM 주파수
MIN_PULSE   = 0.5     # ms (0°)
MAX_PULSE   = 2.5     # ms (180°)
PERIOD_MS   = 20.0    # 1/50Hz = 20ms


def angle_to_duty(angle: float) -> float:
    """각도 (0~180) → PWM duty cycle (%)"""
    angle = max(0.0, min(180.0, angle))
    pulse_ms = MIN_PULSE + (MAX_PULSE - MIN_PULSE) * angle / 180.0
    return (pulse_ms / PERIOD_MS) * 100.0


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(SIGNAL_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SIGNAL_PIN, FREQ_HZ)
    pwm.start(0)
    print(f"[INIT] GPIO {SIGNAL_PIN} | {FREQ_HZ}Hz | {MIN_PULSE}~{MAX_PULSE}ms")
    return pwm


def move_to(pwm, angle: float, hold_s: float = 0.6):
    """목표 각도로 이동 후 신호 차단 (발열 방지)"""
    duty = angle_to_duty(angle)
    print(f"[MOVE] {angle:.1f}° → duty {duty:.2f}%")
    pwm.ChangeDutyCycle(duty)
    time.sleep(hold_s)
    pwm.ChangeDutyCycle(0)  # 신호 차단


def interactive_mode(pwm):
    """각도를 직접 입력하는 대화형 모드"""
    print("\n대화형 모드 시작. 'q' 입력 시 종료.")
    print("입력 예: 90  /  0  /  180  /  45.5")
    while True:
        try:
            raw = input("\n각도 입력 (0~180, q=종료): ").strip()
            if raw.lower() == 'q':
                break
            angle = float(raw)
            if not 0 <= angle <= 180:
                print("  ⚠ 범위 초과: 0~180 사이로 입력하세요.")
                continue
            move_to(pwm, angle)
        except ValueError:
            print("  숫자로 입력하세요.")
        except KeyboardInterrupt:
            break


def cleanup(pwm):
    pwm.stop()
    GPIO.cleanup()
    print("\n[CLEANUP] GPIO 정리 완료")


if __name__ == "__main__":
    pwm = setup()

    try:
        if len(sys.argv) == 1:
            # 인자 없음 → 대화형 모드
            interactive_mode(pwm)

        elif len(sys.argv) == 2:
            # 각도 1개 → 이동 후 종료
            angle = float(sys.argv[1])
            move_to(pwm, angle)
            print(f"[DONE] {angle}° 완료")

        else:
            # 각도 여러 개 → 순서대로 이동
            angles = [float(a) for a in sys.argv[1:]]
            print(f"[SEQ] {angles} 순서대로 이동")
            for angle in angles:
                move_to(pwm, angle)
                time.sleep(0.3)
            print("[DONE] 시퀀스 완료")

    finally:
        cleanup(pwm)
