import RPi.GPIO as GPIO
import time

# 핀 번호 설정 (BCM 모드)
DIR_PIN = 17
STEP_PIN = 12

# 속도 및 범위 설정
DELAY = 0.005    # 작을수록 빠름
MAX_POS = 3000   # 액추에이터의 최대 이동 거리 (실험하며 조절하세요)
MIN_POS = 0      # 시작점
current_pos = 0  # 현재 위치 저장용

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(STEP_PIN, GPIO.OUT)
    print("시스템 준비 완료. 모터를 맨 뒤로 밀어놓은 상태에서 시작하세요.")

def move_to_limit(direction):
    global current_pos
    
    # 방향 설정
    GPIO.output(DIR_PIN, direction)
    
    if direction == GPIO.HIGH: # 전진 (1 입력 시)
        if current_pos >= MAX_POS:
            print("경고: 이미 최대 지점(MAX)입니다.")
            return
        
        print("전진 시작...")
        while current_pos < MAX_POS:
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(DELAY)
            current_pos += 1
            
    else: # 후진 (0 입력 시)
        if current_pos <= MIN_POS:
            print("경고: 이미 최소 지점(MIN)입니다.")
            return
        
        print("후진 시작...")
        while current_pos > MIN_POS:
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(DELAY)
            current_pos -= 1

    print(f"이동 완료. 현재 위치: {current_pos}")

try:
    setup()
    while True:
        cmd = input("\n명령 입력 (1:전진, 0:후진, q:종료): ").lower()

        if cmd == '1':
            move_to_limit(GPIO.HIGH)
        elif cmd == '0':
            move_to_limit(GPIO.LOW)
        elif cmd == 'q':
            print("프로그램을 종료합니다.")
            break
        else:
            print("잘못된 입력입니다.")

except KeyboardInterrupt:
    print("\n사용자가 중단했습니다.")

finally:
    GPIO.cleanup()
    print("GPIO 설정이 초기화되었습니다.")
