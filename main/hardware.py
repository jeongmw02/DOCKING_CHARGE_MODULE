# hardware.py — GPIO 초기화, 액추에이터·승인 헬퍼 함수

import time
import config
from constants import *
import shared

# ── pigpio (스테퍼 + 서보 정밀 제어) ─────────────────────────
_pi = None
try:
    import pigpio
    _pi = pigpio.pi()
    if not _pi.connected:
        print("[WARN] pigpiod 데몬 미실행 → sudo pigpiod 실행 필요")
        _pi = None
except ImportError:
    print("[WARN] pigpio 없음 → 시뮬레이션 모드")

# ── RPi.GPIO (전자석 PWM용) ───────────────────────────────────
try:
    import RPi.GPIO as GPIO
    GPIO_OK = True
except ImportError:
    GPIO_OK = False
    print("[WARN] RPi.GPIO 없음 → GPIO 시뮬레이션 모드")

import cv2
import cv2.aruco as aruco

def _open_usb_camera():
    for idx in range(6):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # 최신 프레임만 유지
                print(f"[CAM] USB 카메라 발견: /dev/video{idx}")
                return cap
            cap.release()
    return None

CAMERA_OK = True

try:
    import board, busio, adafruit_vl53l0x
    TOF_OK = True
except ImportError:
    TOF_OK = False
    print("[WARN] adafruit_vl53l0x 없음 → ToF 비활성")

# ════════════════════════════════════════════════════════
# ── GPIO / pigpio 초기화 ─────────────────────────────────
# ════════════════════════════════════════════════════════
if GPIO_OK:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    # 전자석 — RPi.GPIO PWM
    GPIO.setup(config.ELECTROMAGNET_PIN, GPIO.OUT, initial=GPIO.LOW)
    _pwm = GPIO.PWM(config.ELECTROMAGNET_PIN, 1000)
    _pwm.start(0)
    # LED
    GPIO.setup(config.LED_PIN, GPIO.OUT, initial=GPIO.HIGH)
    # 스테퍼 — RPi.GPIO
    GPIO.setup(config.STEPPER_STEP_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(config.STEPPER_DIR_PIN,  GPIO.OUT, initial=GPIO.HIGH)
    if config.STEPPER_EN_PIN is not None:
        GPIO.setup(config.STEPPER_EN_PIN, GPIO.OUT, initial=GPIO.HIGH)  # 비활성
    # 서보 — pigpio 우선, 없으면 RPi.GPIO PWM
    if _pi:
        _pi.set_mode(SERVO_CHARGING_PIN, pigpio.OUTPUT)
        _pi.set_servo_pulsewidth(SERVO_CHARGING_PIN, 0)
        _servo_pwm = None
    else:
        GPIO.setup(SERVO_CHARGING_PIN, GPIO.OUT, initial=GPIO.LOW)
        _servo_pwm = GPIO.PWM(SERVO_CHARGING_PIN, SERVO_FREQ_HZ)
        _servo_pwm.start(0)
else:
    _pwm = None
    _servo_pwm = None


# ════════════════════════════════════════════════════════
# ── 승인 헬퍼 ────────────────────────────────────────────
# ════════════════════════════════════════════════════════
def _request_approval(next_state: str):
    with shared._lock:
        if shared._pending_transition == next_state:
            return
        shared._pending_transition = next_state
        shared._approved = False
    print(f"\n[SM] PENDING [{next_state}] — Enter or APPROVE")

def _check_approved() -> bool:
    with shared._lock:
        if shared._approved:
            shared._approved = False
            shared._pending_transition = None
            return True
    return False

def _cancel_approval():
    with shared._lock:
        if shared._pending_transition is not None:
            print(f"\n[SM] CANCELLED [{shared._pending_transition}]")
        shared._pending_transition = None
        shared._approved = False


# ════════════════════════════════════════════════════════
# ── 액추에이터 헬퍼 ──────────────────────────────────────
# ════════════════════════════════════════════════════════
def _set_magnet(on: bool):
    with shared._lock:
        if shared._magnet_on == on:
            return
    if on:
        if GPIO_OK:
            _pwm.ChangeDutyCycle(config.ELECTROMAGNET_PULL_DUTY)
            time.sleep(config.ELECTROMAGNET_PULL_MS / 1000.0)
            _pwm.ChangeDutyCycle(config.ELECTROMAGNET_HOLD_DUTY)
        with shared._lock:
            shared._magnet_on = True
        print("[MAG] ON")
    else:
        if GPIO_OK:
            _pwm.ChangeDutyCycle(0)
        with shared._lock:
            shared._magnet_on = False
        print("[MAG] OFF")


def _set_motor_target(steps: int):
    with shared._lock:
        if shared._motor_target == steps:
            return
        shared._motor_target = steps
    print("[MOT] target ->", steps, "steps")


def _motor_enable(enable):
    if not GPIO_OK or config.STEPPER_EN_PIN is None:
        return
    GPIO.output(config.STEPPER_EN_PIN, GPIO.LOW if enable else GPIO.HIGH)


def _step_motor_once(direction, delay):
    half = delay / 2
    if _pi:
        _pi.write(config.STEPPER_DIR_PIN, 0 if direction > 0 else 1)
        _pi.write(config.STEPPER_STEP_PIN, 1)
        time.sleep(half)
        _pi.write(config.STEPPER_STEP_PIN, 0)
        time.sleep(half)
    elif GPIO_OK:
        GPIO.output(config.STEPPER_DIR_PIN, GPIO.LOW if direction > 0 else GPIO.HIGH)
        GPIO.output(config.STEPPER_STEP_PIN, GPIO.HIGH)
        time.sleep(half)
        GPIO.output(config.STEPPER_STEP_PIN, GPIO.LOW)
        time.sleep(half)
    else:
        time.sleep(delay)


def _set_servo(angle_deg):
    with shared._lock:
        if shared._servo_angle == angle_deg:
            return
    if _pi:
        pulse_us = int(500 + (2000.0 * angle_deg / 180.0))
        pulse_us = max(500, min(2500, pulse_us))
        _pi.set_servo_pulsewidth(SERVO_CHARGING_PIN, pulse_us)
        time.sleep(0.8)
    elif GPIO_OK and _servo_pwm is not None:
        duty = 2.5 + (10.0 * angle_deg / 180.0)
        _servo_pwm.ChangeDutyCycle(duty)
        time.sleep(0.8)
    with shared._lock:
        shared._servo_angle = angle_deg
    print("[SVO] ->", angle_deg, "deg")
