import RPi.GPIO as GPIO
import time

# 핀 번호 설정 (BCM 모드 기준)
DIR_PIN = 17
STEP_PIN = 12

# 스텝 간의 지연 시간 (초 단위). 이 값을 줄이면 속도가 빨라집니다.
# 너무 값을 작게(빠르게) 주면 모터가 윙윙 소리만 나고 움직이지 않을 수 있습니다.
DELAY = 0.005 

# 움직일 스텝 수 설정
STEPS = 1500 

def setup():
    # GPIO 모드를 BCM으로 설정
    GPIO.setmode(GPIO.BCM)
    # 경고 메시지 비활성화
    GPIO.setwarnings(False)
    
    # 핀을 출력 모드로 설정
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(STEP_PIN, GPIO.OUT)

def move_motor(direction, steps, delay):
    # 방향 설정 (True: 정방향, False: 역방향)
    GPIO.output(DIR_PIN, direction)
    
    for _ in range(steps):
        # STEP 핀에 HIGH 신호를 주어 한 스텝 이동
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        # STEP 핀을 다시 LOW로 내림
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(delay)

try:
    setup()
    print("리니어 모터 테스트를 시작합니다.")
    
    print("앞으로 이동 중...")
    move_motor(GPIO.HIGH, STEPS, DELAY)
    
    time.sleep(1) # 1초 대기
    
    print("뒤로 이동 중...")
    move_motor(GPIO.LOW, STEPS, DELAY)

except KeyboardInterrupt:
    print("\n사용자에 의해 프로그램이 중단되었습니다.")

finally:
    # 프로그램 종료 시 GPIO 핀 상태 초기화 (매우 중요)
    GPIO.cleanup()
    print("GPIO 정리가 완료되었습니다. 프로그램 종료.")
