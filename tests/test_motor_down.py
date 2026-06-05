import RPi.GPIO as GPIO
import time

DIR_PIN = 17
STEP_PIN = 12
STEPS = 12500
MIN_DELAY = 0.001
MAX_DELAY = 0.004
ACCEL_STEPS = 300

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(STEP_PIN, GPIO.OUT)

def step_once(delay):
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(delay)
    GPIO.output(STEP_PIN, GPIO.LOW)
    time.sleep(delay)

def move_motor(direction, steps, min_delay, max_delay, accel_steps):
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

try:
    setup()
    print("아래로 이동 중...")
    move_motor(GPIO.HIGH, STEPS, MIN_DELAY, MAX_DELAY, ACCEL_STEPS)
except KeyboardInterrupt:
    print("\n중단됨")
finally:
    GPIO.cleanup()
    print("종료")
