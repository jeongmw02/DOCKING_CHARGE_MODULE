import RPi.GPIO as GPIO
import time
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import config

DIR_PIN    = config.STEPPER_DIR_PIN   # GPIO17
STEP_PIN   = config.STEPPER_STEP_PIN  # GPIO12
EN_PIN     = config.STEPPER_EN_PIN    # GPIO27
EMAG_PIN   = config.ELECTROMAGNET_PIN # GPIO18

STEPS       = 11000
MIN_DELAY   = 0.001
MAX_DELAY   = 0.002
ACCEL_STEPS = 300

def setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DIR_PIN,  GPIO.OUT)
    GPIO.setup(STEP_PIN, GPIO.OUT)
    GPIO.setup(EN_PIN,   GPIO.OUT)
    GPIO.setup(EMAG_PIN, GPIO.OUT, initial=GPIO.LOW)

def step_once(delay):
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(delay)
    GPIO.output(STEP_PIN, GPIO.LOW)
    time.sleep(delay)

def move_motor(direction, steps, min_delay, max_delay, accel_steps):
    GPIO.output(EN_PIN,  GPIO.LOW)   # 드라이버 활성화 (EN=LOW)
    GPIO.output(DIR_PIN, direction)
    for i in range(steps):
        if i < accel_steps:
            delay = max_delay - (max_delay - min_delay) * (i / accel_steps)
        elif i > steps - accel_steps:
            delay = max_delay - (max_delay - min_delay) * ((steps - i) / accel_steps)
        else:
            delay = min_delay

        if i % 1000 == 0:
            print(f"현재 스텝: {i} / {steps}")

        step_once(delay)
    GPIO.output(EN_PIN, GPIO.HIGH)   # 드라이버 비활성화

try:
    setup()

    # 전자석 ON
    print("전자석 ON")
    GPIO.output(EMAG_PIN, GPIO.HIGH)

    print("위로 이동 중...")
    move_motor(GPIO.LOW, STEPS, MIN_DELAY, MAX_DELAY, ACCEL_STEPS)
    print("완료")

except KeyboardInterrupt:
    print("\n중단됨")
finally:
    GPIO.output(EMAG_PIN, GPIO.LOW)  # 전자석 OFF
    GPIO.cleanup()
    print("종료")
