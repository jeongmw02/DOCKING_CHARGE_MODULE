# tests/test_dfr0438_led.py
# DFR0438 LED 모듈 테스트 — GPIO27 (pin 13)
#
# DFR0438: DFRobot Gravity Digital LED Module
#   HIGH → ON, LOW → OFF
#
# 사용법:
#   python3 test_dfr0438_led.py            # 대화형 모드
#   python3 test_dfr0438_led.py on         # 켜기
#   python3 test_dfr0438_led.py off        # 끄기
#   python3 test_dfr0438_led.py blink 5    # 5회 점멸
#   python3 test_dfr0438_led.py blink 5 0.2  # 5회, 0.2초 간격

import sys
import time
import RPi.GPIO as GPIO

# ── 핀 설정 ───────────────────────────────────────────────
LED_PIN = 27   # GPIO27 (pin 13)


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW)
    print(f"[INIT] DFR0438 LED | GPIO{LED_PIN}")


def led_on():
    GPIO.output(LED_PIN, GPIO.HIGH)
    print("[LED] ON")


def led_off():
    GPIO.output(LED_PIN, GPIO.LOW)
    print("[LED] OFF")


def blink(count: int = 5, interval: float = 0.5):
    """count회 점멸. interval = 켜짐/꺼짐 각각의 유지 시간(초)"""
    print(f"[BLINK] {count}회 | {interval}s 간격")
    for i in range(count):
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(interval)
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(interval)
        print(f"  {i+1}/{count}")
    print("[BLINK] 완료")


def interactive_mode():
    print("\n조작법: on / off / blink [횟수] [간격] / q=종료")
    while True:
        try:
            raw = input("\n입력: ").strip().lower().split()
            if not raw:
                continue
            cmd = raw[0]

            if cmd == 'q':
                break
            elif cmd == 'on':
                led_on()
            elif cmd == 'off':
                led_off()
            elif cmd == 'blink':
                count    = int(raw[1])   if len(raw) > 1 else 5
                interval = float(raw[2]) if len(raw) > 2 else 0.5
                blink(count, interval)
            else:
                print("  on / off / blink / q 로 입력하세요.")

        except (ValueError, IndexError):
            print("  입력 오류. 예: blink 10 0.3")
        except KeyboardInterrupt:
            break


def cleanup():
    led_off()
    GPIO.cleanup()
    print("[CLEANUP] 완료")


if __name__ == "__main__":
    setup()
    try:
        if len(sys.argv) == 1:
            interactive_mode()

        elif sys.argv[1] == 'on':
            led_on()
            input("Enter 누르면 끕니다...")
            led_off()

        elif sys.argv[1] == 'off':
            led_off()

        elif sys.argv[1] == 'blink':
            count    = int(sys.argv[2])   if len(sys.argv) > 2 else 5
            interval = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
            blink(count, interval)

        else:
            print("사용법: on / off / blink [횟수] [간격(초)]")

    except KeyboardInterrupt:
        print("\n[STOP]")
    finally:
        cleanup()
