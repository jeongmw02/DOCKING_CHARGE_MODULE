# tests/test_electromagnet.py
# 전자석 수동(입력형) ON/OFF 제어 테스트 스크립트
# 실행: python tests/test_electromagnet.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time

def test_electromagnet_interactive():
    import RPi.GPIO as GPIO
    import config

    # 초기화
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(config.ELECTROMAGNET_PIN, GPIO.OUT, initial=GPIO.LOW)
    pwm = GPIO.PWM(config.ELECTROMAGNET_PIN, 1000)
    pwm.start(0)

    print("=" * 45)
    print("전자석 대화형 테스트 시작")
    print("명령어: 'ON' (100% 켜기), 'OFF' (끄기), 'Q' (종료)")
    print("=" * 45)

    try:
        while True:
            # 사용자 입력 받기 (양쪽 공백 제거 및 대문자 변환)
            cmd = input("\n명령을 입력하세요 (ON/OFF/Q): ").strip().upper()
            
            if cmd == 'ON':
                print(">> 전자석 ON (Duty Cycle: 100%)")
                pwm.ChangeDutyCycle(100)
            elif cmd == 'OFF':
                print(">> 전자석 OFF (Duty Cycle: 0%)")
                pwm.ChangeDutyCycle(0)
            elif cmd == 'Q' or cmd == 'QUIT':
                print(">> 테스트를 종료합니다.")
                break
            else:
                print(">> 잘못된 명령어입니다. 'ON', 'OFF', 'Q' 중 하나를 입력하세요.")

    except KeyboardInterrupt:
        # Ctrl+C 로 강제 종료 시 안전하게 처리
        print("\n>> 강제 종료(Ctrl+C)가 감지되었습니다.")
    
    finally:
        # 오류가 나거나 정상 종료되거나 무조건 실행되는 안전장치
        pwm.stop()
        GPIO.cleanup(config.ELECTROMAGNET_PIN)
        print(">> 하드웨어 제어권 반환 및 정리 완료.")

if __name__ == "__main__":
    test_electromagnet_interactive()
