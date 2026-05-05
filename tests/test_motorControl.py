import RPi.GPIO as GPIO
import time

# 핀 번호 설정 (BCM 모드 기준)
DIR_PIN = 17
STEP_PIN = 12

# 스텝 설정
DELAY = 0.005 
STEPS = 500  # 한 번 입력할 때 움직일 거리 (필요에 따라 조절하세요)

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(STEP_PIN, GPIO.OUT)

def move_motor(direction, steps, delay):
    GPIO.output(DIR_PIN, direction)
    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(delay)

try:
    setup()
    print("--- 리니어 모터 제어 프로그램 ---")
    print("1: 앞으로 이동 / 0: 뒤로 이동 / q: 종료")
    
    while True:
        # 사용자로부터 입력 받기
        user_input = input("명령을 입력하세요 (1/0/q): ").lower()

        if user_input == '1':
            print("앞으로 이동 중...")
            move_motor(GPIO.HIGH, STEPS, DELAY)
        
        elif user_input == '0':
            print("뒤로 이동 중...")
            move_motor(GPIO.LOW, STEPS, DELAY)
            
        elif user_input == 'q':
            print("프로그램을 종료합니다.")
            break
            
        else:
            print("잘못된 입력입니다. 1, 0 또는 q를 입력하세요.")

except KeyboardInterrupt:
    print("\n사용자에 의해 프로그램이 중단되었습니다.")

finally:
    GPIO.cleanup()
    print("GPIO 정리가 완료되었습니다.")
