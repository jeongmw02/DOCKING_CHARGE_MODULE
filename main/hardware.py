# hardware.py — GPIO 초기화, 액추에이터·승인 헬퍼 함수

import time
import config
from constants import *
import shared

def _angle_to_duty(angle_deg: float) -> float:
    """서보 각도(0~180°) → PWM 듀티(%) 변환.
    0.5ms ~ 2.5ms 펄스 범위, 20ms 주기 기준."""
    angle = max(0.0, min(180.0, angle_deg))
    pulse_ms = 0.5 + (2.0 * angle / 180.0)
    return pulse_ms / 20.0 * 100.0


# ── 하드웨어 임포트 ─────────────────────────────────────
try:
    import RPi.GPIO as GPIO
    GPIO_OK = True
except ImportError:
    GPIO_OK = False
    print("[WARN] RPi.GPIO 없음 → GPIO 시뮬레이션 모드")

import cv2
import cv2.aruco as aruco

def _open_usb_camera():
    """V4L2 인덱스 0~5 중 첫 번째로 열리는 카메라를 반환."""
    for idx in range(6):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                print(f"[CAM] USB 카메라 발견: /dev/video{idx}")
                return cap
            cap.release()
    return None

CAMERA_OK = True   # cv2는 항상 있으므로 True; 장치 없으면 thread 내부에서 처리

try:
    import board, busio, adafruit_vl53l0x
    TOF_OK = True
except ImportError:
    TOF_OK = False
    print("[WARN] adafruit_vl53l0x 없음 → ToF 비활성")

# ════════════════════════════════════════════════════════
# ── GPIO 초기화 ─────────────────────────────────────────
# ════════════════════════════════════════════════════════
if GPIO_OK:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(config.ELECTROMAGNET_PIN,  GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(config.STEPPER_STEP_PIN,   GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(config.STEPPER_DIR_PIN,    GPIO.OUT, initial=GPIO.HIGH)
    _pwm = GPIO.PWM(config.ELECTROMAGNET_PIN, 1000)
    _pwm.start(0)

    # DFR0438 LED — 항상 ON
    GPIO.setup(config.LED_PIN, GPIO.OUT, initial=GPIO.HIGH)

    # MG992 충전 서보 (50Hz PWM)
    GPIO.setup(SERVO_CHARGING_PIN, GPIO.OUT, initial=GPIO.LOW)
    _servo_pwm = GPIO.PWM(SERVO_CHARGING_PIN, SERVO_FREQ_HZ)
    _servo_pwm.start(0)
else:
    _servo_pwm = None
    _pwm = None

def _request_approval(next_state: str):
    """다음 상태 전이 전 승인 요청. _pending_transition을 설정하고 반환."""
    with shared._lock:
        if shared._pending_transition == next_state:
            return  # 이미 같은 전이 대기 중
        shared._pending_transition = next_state
        shared._approved = False
    print(f"\n[SM] ⏸  [{next_state}] 전이 대기 — 웹 UI에서 APPROVE 클릭 필요")

def _check_approved() -> bool:
    """승인됐으면 True 반환 + 플래그 초기화."""
    with shared._lock:
        if shared._approved:
            shared._approved = False
            shared._pending_transition = None
            return True
    return False

def _cancel_approval():
    """대기 중인 승인 취소 (조건 소실 또는 REJECT 시)."""
    with shared._lock:
        if shared._pending_transition is not None:
            print(f"\n[SM] ✗  [{shared._pending_transition}] 전이 취소")
        shared._pending_transition = None
        shared._approved = False


def _set_magnet(on: bool):
    """전자석 상태 설정. 이미 해당 상태면 무시."""
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
    """모터 목표 스텝 설정. 이미 같은 값이면 무시."""
    with shared._lock:
        if shared._motor_target == steps:
            return
        shared._motor_target = steps
    print("[MOT] target ->", steps, "steps")

def _set_servo(angle_deg: float):
    """MG992 서보 회전. 이미 동일 각도면 무시.
    PWM 신호를 짧게 인가 후 차단해 발열·지터를 방지한다."""
    with shared._lock:
        if shared._servo_angle == angle_deg:
            return
    if GPIO_OK and _servo_pwm is not None:
        duty = _angle_to_duty(angle_deg)
        _servo_pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)               # 서보 이동 시간 확보
        # 신호 유지 — 차단 시 토크 소실로 복귀되는 문제 방지
    with shared._lock:
        shared._servo_angle = angle_deg
    print("[SVO] ->", angle_deg, "deg")