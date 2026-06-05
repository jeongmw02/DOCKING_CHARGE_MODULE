# tests/test_mg995r_pigpio.py
# MG995R 연속 회전 서보 제어 (하드웨어 PWM 기반 Open-loop 제어)
# 사전 요구사항: sudo apt-get install pigpio && sudo pigpiod 실행 필수

import sys
import time
import pigpio

# ── 핀 및 하드웨어 설정 ─────────────────────────────────────────
SIGNAL_PIN = 13

# ── 펄스 범위 (단위: us) ────────────────────────────────────────
# [중요 캘리브레이션 포인트]
# 코드 실행 직후 모터가 한쪽으로 미세하게 회전(Creeping)한다면, 
# 1500 값을 1480 ~ 1520 사이에서 5us 단위로 조절하여 완전한 정지점(Deadband)을 찾으십시오.
STOP_PULSE_US = 1500  

MIN_PULSE_US  = 500   # +방향 최대 속도 지령치
MAX_PULSE_US  = 2500  # -방향 최대 속도 지령치

# ── 속도/각도 추정 제어 파라미터 ────────────────────────────────
DRIVE_SPEED  = 50        # % (구동 속도, 제어기 포화 방지를 위해 50% 권장)
DEG_PER_SEC  = 160.0     # 50% 속도 기준 초당 회전 각도 실측값 (모터별 편차 존재)
STEP_DEG     = 10        # 기본 증감 단위 (도)


def speed_to_pulsewidth(speed: float) -> int:
    """
    제어 지령(속도 비율, -100 ~ 100%)을 하드웨어 펄스 폭(us)으로 변환합니다.
    """
    speed = max(-100.0, min(100.0, speed))
    pulse_us = STOP_PULSE_US - (speed / 100.0) * (STOP_PULSE_US - MIN_PULSE_US)
    return int(pulse_us)


def setup():
    pi = pigpio.pi()
    if not pi.connected:
        print("[ERROR] pigpio 데몬에 연결 실패. 'sudo pigpiod' 명령이 실행되었는지 확인하십시오.")
        sys.exit(1)
    
    # 초기화: 구동 시스템에 정지 펄스 인가
    pi.set_servo_pulsewidth(SIGNAL_PIN, STOP_PULSE_US)
    print(f"[INIT] GPIO {SIGNAL_PIN} | 하드웨어 PWM(pigpio) 활성화 | 중립 펄스 설정값: {STOP_PULSE_US}us")
    return pi


def rotate_deg(pi, degrees: float):
    """
    지정된 각도만큼 회전합니다. (위치 피드백이 없는 Open-loop 시간 제어 기반)
    """
    if degrees == 0:
        return

    direction = 1 if degrees > 0 else -1
    duration  = abs(degrees) / DEG_PER_SEC
    speed     = DRIVE_SPEED * direction

    print(f"  → {'정방향' if direction > 0 else '역방향'} {abs(degrees):.1f}° (인가 시간: {duration:.3f}s)")
    
    # 목표 속도 펄스 인가
    pi.set_servo_pulsewidth(SIGNAL_PIN, speed_to_pulsewidth(speed))
    time.sleep(duration)
    
    # 목표 시간 도달 후 정지 펄스 인가
    pi.set_servo_pulsewidth(SIGNAL_PIN, STOP_PULSE_US)
    time.sleep(0.1)  # 모터 관성 감쇠 및 제어기 안정화 대기


def cleanup(pi):
    # PWM 신호 전송을 완전히 차단하여 서보모터 토크 해제
    pi.set_servo_pulsewidth(SIGNAL_PIN, 0) 
    pi.stop()
    print("[CLEANUP] 하드웨어 자원 반환 및 연결 해제 완료")


if __name__ == "__main__":
    pi = setup()
    total_deg = 0.0

    print(f"\n[Open-loop Control Mode] 조작법:")
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
                    print("  [WARN] '+' / '-' / 숫자 / 'r' / 'q' 중 유효한 커맨드를 입력하십시오.")
                    continue

            rotate_deg(pi, deg)
            total_deg += deg
            print(f"  누적: {total_deg:.1f}°")

    except KeyboardInterrupt:
        print("\n[STOP] 사용자에 의한 시스템 인터럽트 수신")
    finally:
        cleanup(pi)
