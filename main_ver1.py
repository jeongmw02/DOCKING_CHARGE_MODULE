"""
main_ver1.py
============
CubeSat 도킹·충전 모듈 - 2차 시연 메인 스크립트

6-State Machine:
  State 1  PRE_DOCKING  : 초기 대기 — 전자석 OFF, 모터 step 0
  State 2  TARGET_LOCK  : ArUco 마커 감지 — 모터 step 0→3000
  State 3  SOFT_CAPTURE : ToF ≤ 300mm (3s) — 전자석 ON, 모터 step 3000 유지
  State 4  HARD_LOCK    : ToF ≤ 15mm  (5s) — 전자석 ON,  모터 step 3000→0
  State 5  DOCKED       : 모터 step 0 도달 후 2s — 전자석 OFF, step 0 유지
  State 6  CHARGING     : DOCKED 후 2s — 전자석 OFF, 모터 step 0 유지, MG992 45°

State 전이:
  PRE_DOCKING  → TARGET_LOCK  : ArUco 마커 감지
  TARGET_LOCK  → PRE_DOCKING  : 마커 1.5s 소실 (미진입 시에만)
  TARGET_LOCK  → SOFT_CAPTURE : ToF ≤ 300mm 3s 지속
  SOFT_CAPTURE → HARD_LOCK    : ToF ≤ 15mm  5s 지속
  HARD_LOCK    → DOCKED       : 모터 step 0 도달 후 2s
  DOCKED       → CHARGING     : DOCKED 진입 후 2s (MG992 서보 45° 회전)

실행: python3 main_ver1.py
접속: http://<pi_ip>:5000
"""

import time
import math
import threading
import config
from flask import Flask, Response, jsonify

# ── MG992 서보(충전 메커니즘) 설정 ─────────────────────────
# 도킹 완료 후 충전 단자 연결을 위해 45° 회전하는 서보
SERVO_CHARGING_PIN   = 19      # GPIO19 (PWM1) — 기존 핀과 충돌 없음
SERVO_FREQ_HZ        = 50      # 표준 RC 서보 50Hz
SERVO_CHARGING_ANGLE = 45      # 충전 위치 각도 (deg)
SERVO_STOWED_ANGLE   = 0       # 대기 위치 각도 (deg)
CHARGING_WAIT_S      = 2.0     # DOCKED → CHARGING 전이 대기 시간 (s)


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

try:
    from picamera2 import Picamera2
    import cv2
    import cv2.aruco as aruco
    CAMERA_OK = True
except ImportError:
    CAMERA_OK = False
    import cv2
    print("[WARN] picamera2 없음 → 카메라 비활성")

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

    # MG992 충전 서보 (50Hz PWM)
    GPIO.setup(SERVO_CHARGING_PIN, GPIO.OUT, initial=GPIO.LOW)
    _servo_pwm = GPIO.PWM(SERVO_CHARGING_PIN, SERVO_FREQ_HZ)
    _servo_pwm.start(0)
else:
    _servo_pwm = None

# ════════════════════════════════════════════════════════
# ── 전역 공유 상태 (모든 스레드가 _lock으로 보호) ──────────
# ════════════════════════════════════════════════════════
_lock = threading.Lock()

# 센서
_distance_mm     = -1.0
_marker_detected = False
_marker_angle    = 0.0
_marker_offset_x = 0.0
_marker_offset_y = 0.0

# 액추에이터
_magnet_on    = False
_motor_steps  = 0      # 현재 스텝 (motor_thread가 갱신)
_motor_target = 0      # 목표 스텝 (state_machine이 설정)
_servo_angle  = 0      # MG992 충전 서보 현재 각도 (deg)

# 상태머신
_dock_state = "PRE_DOCKING"
_start_time = time.time()

# 카메라 프레임 (camera_thread → gen_frames)
_last_frame = None

app = Flask(__name__)

# ════════════════════════════════════════════════════════
# ── 액추에이터 제어 헬퍼 ────────────────────────────────
# ════════════════════════════════════════════════════════

def _set_magnet(on: bool):
    """전자석 상태 설정. 이미 해당 상태면 무시."""
    global _magnet_on
    with _lock:
        if _magnet_on == on:
            return
    if on:
        if GPIO_OK:
            _pwm.ChangeDutyCycle(config.ELECTROMAGNET_PULL_DUTY)
            time.sleep(config.ELECTROMAGNET_PULL_MS / 1000.0)
            _pwm.ChangeDutyCycle(config.ELECTROMAGNET_HOLD_DUTY)
        with _lock:
            _magnet_on = True
        print("[MAG] 전자석 ON")
    else:
        if GPIO_OK:
            _pwm.ChangeDutyCycle(0)
        with _lock:
            _magnet_on = False
        print("[MAG] 전자석 OFF")

def _set_motor_target(steps: int):
    """모터 목표 스텝 설정. 이미 같은 값이면 무시."""
    global _motor_target
    with _lock:
        if _motor_target == steps:
            return
        _motor_target = steps
    print(f"[MOT] 목표 → {steps} steps")

def _set_servo(angle_deg: float):
    """MG992 서보 회전. 이미 동일 각도면 무시.
    PWM 신호를 짧게 인가 후 차단해 발열·지터를 방지한다."""
    global _servo_angle
    with _lock:
        if _servo_angle == angle_deg:
            return
    if GPIO_OK and _servo_pwm is not None:
        duty = _angle_to_duty(angle_deg)
        _servo_pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)               # 서보 이동 시간 확보
        _servo_pwm.ChangeDutyCycle(0) # 신호 차단 (발열 방지)
    with _lock:
        _servo_angle = angle_deg
    print(f"[SVO] MG992 → {angle_deg}°")

# ════════════════════════════════════════════════════════
# ── Thread 1: ToF 센서 ──────────────────────────────────
# ════════════════════════════════════════════════════════

def tof_thread():
    global _distance_mm
    if not TOF_OK:
        print("[ToF] 라이브러리 없음, 스레드 종료")
        return
    try:
        i2c    = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_vl53l0x.VL53L0X(i2c)
        print("[ToF] 초기화 완료")
        while True:
            try:
                dist = float(sensor.range)
                with _lock:
                    _distance_mm = dist
            except Exception:
                with _lock:
                    _distance_mm = -1.0
            time.sleep(0.1)
    except Exception as e:
        print(f"[ToF] 초기화 실패: {e}")

# ════════════════════════════════════════════════════════
# ── Thread 2: 카메라 + ArUco ────────────────────────────
# ════════════════════════════════════════════════════════

def _aruco_angle(corners_single) -> float:
    c  = corners_single[0]
    dx = c[1][0] - c[0][0]
    dy = c[1][1] - c[0][1]
    return math.degrees(math.atan2(dy, dx))

def _aruco_center(corners_single):
    c = corners_single[0]
    return int((c[0][0] + c[2][0]) / 2), int((c[0][1] + c[2][1]) / 2)

def camera_thread():
    global _marker_detected, _marker_angle, _marker_offset_x, _marker_offset_y
    global _last_frame

    if not CAMERA_OK:
        print("[CAM] 카메라 라이브러리 없음, 스레드 종료")
        return

    cam = Picamera2()
    cfg = cam.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    cam.configure(cfg)
    cam.start()
    time.sleep(1)
    print("[CAM] 카메라 시작")

    # ArUco 초기화 (OpenCV 버전 호환)
    try:
        _dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        _param = aruco.DetectorParameters()
        _det   = aruco.ArucoDetector(_dict, _param)
        use_new_api = True
    except AttributeError:
        _dict  = aruco.Dictionary_get(aruco.DICT_4X4_50)
        _param = aruco.DetectorParameters_create()
        _det   = None
        use_new_api = False

    FRAME_INTERVAL  = 1.0 / 15   # 카메라 최대 15 fps
    ARUCO_EVERY_N   = 3          # 3프레임마다 ArUco 감지 (CPU 절감)
    frame_count     = 0
    last_corners    = None
    last_ids        = None
    last_frame_time = 0.0

    while True:
        # FPS 제한
        now = time.time()
        elapsed = now - last_frame_time
        if elapsed < FRAME_INTERVAL:
            time.sleep(FRAME_INTERVAL - elapsed)
        last_frame_time = time.time()

        frame = cam.capture_array()
        h, w, _ = frame.shape
        cx_img, cy_img = w // 2, h // 2
        frame_count += 1

        # ArUco 감지: N프레임마다만 실행
        if frame_count % ARUCO_EVERY_N == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            if use_new_api:
                last_corners, last_ids, _ = _det.detectMarkers(gray)
            else:
                last_corners, last_ids, _ = aruco.detectMarkers(gray, _dict, parameters=_param)

        corners, ids = last_corners, last_ids
        detected = ids is not None and len(ids) > 0

        if detected:
            aruco.drawDetectedMarkers(frame, corners, ids)
            angle  = _aruco_angle(corners[0])
            mx, my = _aruco_center(corners[0])
            off_x  = mx - cx_img
            off_y  = my - cy_img

            with _lock:
                _marker_detected = True
                _marker_angle    = angle
                _marker_offset_x = off_x
                _marker_offset_y = off_y

            # 마커 시각화
            cv2.circle(frame, (mx, my), 6, (0, 255, 80), -1)
            cv2.line(frame, (mx, my), (cx_img, cy_img), (255, 200, 0), 1)
            ang_c = (50, 255, 100) if abs(angle) < 5 else (50, 150, 255)
            cv2.putText(frame, f"Angle: {angle:+.1f} deg",
                        (mx - 65, my - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.55, ang_c, 2)
            cv2.putText(frame, f"dX:{off_x:+.0f} dY:{off_y:+.0f}",
                        (mx - 65, my - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            cv2.putText(frame, "TARGET LOCKED",
                        (mx - 60, my + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 255, 100), 2)
        else:
            with _lock:
                _marker_detected = False

        # 상태 읽기
        with _lock:
            dist  = _distance_mm
            state = _dock_state
            mag   = _magnet_on
            steps = _motor_steps
            tgt   = _motor_target

        # 거리 오버레이
        dist_text  = f"{int(dist)} mm" if dist >= 0 else "N/A"
        dist_color = ((100, 220, 100) if dist > 300 else
                      (50,  200, 255) if dist > 15  else
                      (50,  80,  255)) if dist >= 0 else (120, 120, 120)
        cv2.putText(frame, dist_text, (20, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, dist_color, 2)

        # 상태 오버레이
        state_colors = {
            "PRE_DOCKING":  (160, 160, 160),
            "TARGET_LOCK":  (100, 220, 255),
            "SOFT_CAPTURE": (50,  255, 150),
            "HARD_LOCK":    (50,  150, 255),
            "DOCKED":       (80,  255, 80),
            "CHARGING":     (255, 200, 50),
        }
        sc = state_colors.get(state, (200, 200, 200))
        cv2.putText(frame, state, (20, 88),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, sc, 2)

        # 전자석 + 모터 오버레이
        mag_c = (50, 255, 100) if mag else (100, 100, 200)
        cv2.putText(frame, "MAG: ON " if mag else "MAG: OFF",
                    (20, 122), cv2.FONT_HERSHEY_SIMPLEX, 0.6, mag_c, 2)
        cv2.putText(frame, f"STEP: {steps}/{tgt}",
                    (20, 152), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

        # 크로스헤어
        ch_c = (50, 255, 100) if detected else (130, 130, 130)
        cv2.line(frame, (cx_img - 28, cy_img), (cx_img + 28, cy_img), ch_c, 1)
        cv2.line(frame, (cx_img, cy_img - 28), (cx_img, cy_img + 28), ch_c, 1)
        cv2.circle(frame, (cx_img, cy_img), 32, ch_c, 1)

        with _lock:
            _last_frame = frame.copy()

# ════════════════════════════════════════════════════════
# ── Thread 3: 스테퍼 모터 ───────────────────────────────
# ════════════════════════════════════════════════════════

def motor_thread():
    """_motor_target을 보고 _motor_steps를 1스텝씩 이동."""
    global _motor_steps
    half_delay = config.MOTOR_STEP_DELAY_S / 2

    while True:
        with _lock:
            current = _motor_steps
            target  = _motor_target

        if current == target:
            time.sleep(0.005)
            continue

        direction = 1 if target > current else -1

        if GPIO_OK:
            GPIO.output(config.STEPPER_DIR_PIN,
                        GPIO.HIGH if direction > 0 else GPIO.LOW)
            GPIO.output(config.STEPPER_STEP_PIN, GPIO.HIGH)
            time.sleep(half_delay)
            GPIO.output(config.STEPPER_STEP_PIN, GPIO.LOW)
            time.sleep(half_delay)
        else:
            time.sleep(half_delay * 2)   # 시뮬레이션 딜레이

        with _lock:
            _motor_steps += direction

        # 100스텝마다 콘솔 출력
        with _lock:
            s = _motor_steps
        if s % 100 == 0:
            print(f"[MOT] {s} / {target} steps", end='\r')

# ════════════════════════════════════════════════════════
# ── Thread 4: 상태 머신 ────────────────────────────────
# ════════════════════════════════════════════════════════

def state_machine_thread():
    """
    50ms 루프로 상태 전이를 관리.
    상태 전이는 앞으로만 진행 (역방향 없음).
    모터는 각 상태 진입 시 한 번만 target 설정 — 이후 motor_thread가 담당.
    """
    global _dock_state

    # ── 타이머 변수 ──────────────────────────────────────
    marker_detect_start = None  # PRE_DOCKING → TARGET_LOCK 타이머 (3s)
    dist_300_start      = None  # TARGET_LOCK → SOFT_CAPTURE 타이머
    dist_lock_start     = None  # SOFT_CAPTURE → HARD_LOCK 타이머
    motor_zero_done_t   = None  # HARD_LOCK → DOCKED 타이머
    docked_enter_t      = None  # DOCKED → CHARGING 타이머 (2s)

    def _state(s):
        global _dock_state
        with _lock:
            _dock_state = s
        print(f"\n[SM] ──→ {s}")

    print("[SM] 상태 머신 시작: PRE_DOCKING")

    while True:
        time.sleep(0.05)  # 50ms 루프

        with _lock:
            state  = _dock_state
            dist   = _distance_mm
            marker = _marker_detected
            steps  = _motor_steps
            tgt    = _motor_target

        now = time.time()

        # ═══════════════════════════════════════════════
        # State 1: PRE_DOCKING
        # ═══════════════════════════════════════════════
        if state == "PRE_DOCKING":
            _set_magnet(False)
            _set_motor_target(0)
            _set_servo(SERVO_STOWED_ANGLE)
            dist_300_start    = None
            dist_lock_start   = None
            motor_zero_done_t = None
            docked_enter_t    = None

            if marker:
                if marker_detect_start is None:
                    marker_detect_start = now
                    print("\n[SM] 마커 감지 시작, 3s 카운트...")
                elif now - marker_detect_start >= 3.0:
                    marker_detect_start = None
                    _state("TARGET_LOCK")
            else:
                if marker_detect_start is not None:
                    print("\n[SM] 마커 소실 → 타이머 리셋")
                marker_detect_start = None

        # ═══════════════════════════════════════════════
        # State 2: TARGET_LOCK
        # ═══════════════════════════════════════════════
        elif state == "TARGET_LOCK":
            _set_motor_target(config.MOTOR_TARGET_STEPS)

            if dist >= 0 and dist <= config.SOFT_CAPTURE_DIST_MM:
                if dist_300_start is None:
                    dist_300_start = now
                    print(f"\n[SM] ToF {int(dist)}mm ≤ {config.SOFT_CAPTURE_DIST_MM}mm 감지, "
                          f"{config.SOFT_CAPTURE_HOLD_S}s 카운트 시작")
                elif now - dist_300_start >= config.SOFT_CAPTURE_HOLD_S:
                    dist_300_start = None
                    _set_magnet(True)
                    _state("SOFT_CAPTURE")
            else:
                if dist_300_start is not None:
                    print("\n[SM] 거리 초과 → 타이머 리셋")
                dist_300_start = None

        # ═══════════════════════════════════════════════
        # State 3: SOFT_CAPTURE
        # ═══════════════════════════════════════════════
        elif state == "SOFT_CAPTURE":
            _set_magnet(True)
            _set_motor_target(config.MOTOR_TARGET_STEPS)

            if dist >= 0 and dist <= config.HARD_LOCK_DIST_MM:
                if dist_lock_start is None:
                    dist_lock_start = now
                    print(f"\n[SM] ToF {int(dist)}mm ≤ {config.HARD_LOCK_DIST_MM}mm 감지, "
                          f"{config.HARD_LOCK_HOLD_S}s 카운트 시작")
                elif now - dist_lock_start >= config.HARD_LOCK_HOLD_S:
                    dist_lock_start = None
                    _set_motor_target(0)
                    _state("HARD_LOCK")
            else:
                if dist_lock_start is not None:
                    print("\n[SM] 거리 초과 → 타이머 리셋")
                dist_lock_start = None

        # ═══════════════════════════════════════════════
        # State 4: HARD_LOCK
        # ═══════════════════════════════════════════════
        elif state == "HARD_LOCK":
            _set_magnet(True)

            motor_at_zero = (steps == 0 and tgt == 0)

            if motor_at_zero:
                if motor_zero_done_t is None:
                    motor_zero_done_t = now
                    print(f"\n[SM] 모터 step 0 도달, {config.DOCKED_WAIT_S}s 대기...")
                elif now - motor_zero_done_t >= config.DOCKED_WAIT_S:
                    motor_zero_done_t = None
                    _set_magnet(False)
                    _state("DOCKED")
            else:
                motor_zero_done_t = None

        # ═══════════════════════════════════════════════
        # State 5: DOCKED
        # ═══════════════════════════════════════════════
        elif state == "DOCKED":
            _set_magnet(False)
            _set_motor_target(0)

            if docked_enter_t is None:
                docked_enter_t = now
                print(f"\n[SM] DOCKED 진입, {CHARGING_WAIT_S}s 후 CHARGING 전이...")
            elif now - docked_enter_t >= CHARGING_WAIT_S:
                docked_enter_t = None
                _set_servo(SERVO_CHARGING_ANGLE)
                _state("CHARGING")

        # ═══════════════════════════════════════════════
        # State 6: CHARGING
        # ═══════════════════════════════════════════════
        elif state == "CHARGING":
            _set_magnet(False)
            _set_motor_target(0)
            _set_servo(SERVO_CHARGING_ANGLE)

# ════════════════════════════════════════════════════════
# ── Flask 스트리밍 ──────────────────────────────────────
# ════════════════════════════════════════════════════════

def gen_frames():
    """camera_thread가 갱신하는 _last_frame을 JPEG로 인코딩해 스트리밍."""
    prev_frame = None
    while True:
        with _lock:
            frame = _last_frame

        if frame is None:
            time.sleep(0.05)
            continue

        if frame is prev_frame:
            time.sleep(0.01)
            continue
        prev_frame = frame

        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
               + buf.tobytes() + b'\r\n')

@app.route('/')
def index():
    return HTML_PAGE

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/reset', methods=['POST'])
def api_reset():
    """상태머신을 PRE_DOCKING으로 초기화."""
    global _dock_state, _motor_target, _magnet_on
    _set_magnet(False)
    _set_motor_target(0)
    _set_servo(SERVO_STOWED_ANGLE)
    with _lock:
        _dock_state = "PRE_DOCKING"
    print("\n[SM] ★ 수동 리셋 → PRE_DOCKING")
    return jsonify({"ok": True, "state": "PRE_DOCKING"})

@app.route('/api/status')
def api_status():
    with _lock:
        elapsed = int(time.time() - _start_time)
        h = elapsed // 3600
        m = (elapsed % 3600) // 60
        s = elapsed % 60
        return jsonify({
            "distance_mm":     round(_distance_mm, 1),
            "state":           _dock_state,
            "magnet":          _magnet_on,
            "motor_steps":     _motor_steps,
            "motor_target":    _motor_target,
            "servo_angle":     round(_servo_angle, 1),
            "mission_time":    f"T+ {h:02d}:{m:02d}:{s:02d}",
            "marker_detected": _marker_detected,
            "marker_angle":    round(_marker_angle, 1),
            "marker_offset_x": round(_marker_offset_x, 1),
            "marker_offset_y": round(_marker_offset_y, 1),
        })

# ════════════════════════════════════════════════════════
# ── HTML 페이지 (orbital-command UI 스타일) ─────────────
# ════════════════════════════════════════════════════════

HTML_PAGE = """<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>CUBESAT_OS v2.4</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Space+Mono:wght@400;700&display=swap" rel="stylesheet">
<style>
/* ── Reset & Base ─────────────────────────────── */
*, *::before, *::after { margin:0; padding:0; box-sizing:border-box; }
:root {
  --bg:     #050505;
  --bg2:    #0e0e0e;
  --bg3:    #131313;
  --bg4:    #1c1b1b;
  --bd:     #262626;
  --text:   #e5e2e1;
  --dim:    #c4c7c8;
  --muted:  #555;
  --cyan:   #00eefc;
  --green:  #00FF55;
  --amber:  #ffb74d;
  --red:    #FF0033;
}
body {
  background: var(--bg);
  color: var(--text);
  font-family: 'Space Mono', 'Courier New', monospace;
  height: 100vh;
  display: flex;
  flex-direction: column;
  overflow: hidden;
  -webkit-font-smoothing: antialiased;
}
::-webkit-scrollbar { width:4px; height:4px; }
::-webkit-scrollbar-track { background:#0a0a0a; }
::-webkit-scrollbar-thumb { background:#262626; }
::-webkit-scrollbar-thumb:hover { background:#3a3939; }

/* ── TopBar ─────────────────────────────────── */
#topbar {
  background: var(--bg3);
  border-bottom: 1px solid var(--bd);
  height: 48px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 16px;
  flex-shrink: 0;
  user-select: none;
}
.sys-name { font-size:15px; font-weight:700; letter-spacing:3px; color:#fff; }
.status-pill {
  display: flex; align-items: center; gap: 8px;
  padding: 3px 12px; border: 1px solid;
  font-size: 10px; font-weight: 700; letter-spacing: 2px;
  transition: all 0.4s;
}
.status-dot {
  width: 8px; height: 8px; border-radius: 50%;
  animation: blink 2s infinite;
}
@keyframes blink { 0%,100%{opacity:1} 50%{opacity:0.3} }
.topbar-right { display:flex; align-items:center; gap:20px; }
#clock { font-size:10px; color:var(--muted); letter-spacing:1px; }

/* ── Layout ─────────────────────────────────── */
#body { flex:1; display:flex; overflow:hidden; }

/* ── Sidebar ─────────────────────────────────── */
#sidebar {
  background: var(--bg3);
  border-right: 1px solid var(--bd);
  width: 240px;
  display: flex;
  flex-direction: column;
  flex-shrink: 0;
  user-select: none;
}
.sb-header { padding:16px; border-bottom:1px solid var(--bd); }
.sb-mc  { font-size:9px; font-weight:700; letter-spacing:4px; color:var(--dim); margin-bottom:4px; }
.sb-orbit { font-size:14px; font-weight:700; letter-spacing:2px; color:#fff; }
#sidebar nav { display:flex; flex-direction:column; margin-top:8px; flex:1; }
.nav-btn {
  display:flex; align-items:center; gap:16px;
  padding:14px 16px;
  border:none; border-left:2px solid transparent;
  background:transparent;
  color:var(--dim); cursor:pointer;
  font-family:inherit; font-size:10px; font-weight:700; letter-spacing:3px;
  text-align:left; transition:all 0.15s; width:100%;
}
.nav-btn:hover { color:#fff; background:rgba(32,31,31,0.6); }
.nav-btn.active { color:var(--cyan); border-left-color:var(--cyan); background:rgba(53,53,52,0.3); }
.nav-btn svg { width:18px; height:18px; flex-shrink:0; }
.sb-footer { padding:12px; border-top:1px solid var(--bd); text-align:center; }
.sb-footer span { font-size:8px; letter-spacing:4px; color:#3a3939; }

/* ── Content ─────────────────────────────────── */
#content { flex:1; overflow-y:auto; display:flex; flex-direction:column; min-width:0; }

/* ── Footer ─────────────────────────────────── */
#footer {
  background: var(--bg2);
  border-top: 1px solid var(--bd);
  padding: 10px 16px;
  display: flex;
  align-items: center;
  gap: 16px;
  flex-shrink: 0;
  user-select: none;
}
.f-label { font-size:8px; letter-spacing:4px; color:var(--muted); margin-bottom:3px; }
.f-val   { font-size:20px; color:#fff; letter-spacing:2px; }
.seq-wrap { flex:1; }
.seq-dots { display:flex; align-items:center; justify-content:space-between; position:relative; margin-top:6px; }
.seq-line-bg { position:absolute; top:50%; left:0; right:0; height:1px; background:var(--bd); }
.seq-step { display:flex; flex-direction:column; align-items:center; gap:4px; position:relative; z-index:1; }
.seq-dot {
  width:12px; height:12px; border-radius:50%;
  background:#1a2a1a; border:1px solid #2a3a2a;
  transition:all 0.3s;
}
.seq-dot.done    { background:var(--green); border-color:var(--green); }
.seq-dot.current { background:var(--cyan); border-color:var(--cyan); box-shadow:0 0 6px var(--cyan); }
.seq-lbl { font-size:8px; letter-spacing:1px; color:#333; transition:color 0.3s; }
.seq-lbl.done    { color:var(--green); }
.seq-lbl.current { color:var(--cyan); }
#reset-btn {
  padding:8px 16px; border:1px solid #c0392b; color:#e74c3c;
  background:transparent; cursor:pointer;
  font-family:inherit; font-size:11px; letter-spacing:2px;
  transition:all 0.2s; flex-shrink:0;
}
#reset-btn:hover  { background:#c0392b; color:#fff; }
#reset-btn:active { background:#922b21; }

/* ── Panels ─────────────────────────────────── */
.panel { display:none; flex:1; padding:16px; gap:16px; }
.panel.active { display:flex; }
.panel-hdr { border-bottom:1px solid var(--bd); padding-bottom:8px; margin-bottom:4px; }
.panel-hdr h2 { font-size:11px; font-weight:700; letter-spacing:4px; color:var(--cyan); }
.panel-hdr p  { font-size:9px; color:var(--dim); margin-top:4px; }
.tb { border:1px solid var(--bd); }

/* ── VISUALIZER ─────────────────────────────── */
#tab-visualizer { flex-direction:row; }
#viz-main { flex:2.2; display:flex; flex-direction:column; gap:12px; min-width:0; }
#viz-side { width:260px; flex-shrink:0; display:flex; flex-direction:column; gap:12px; }

.viz-hdr {
  font-size:10px; font-weight:700; letter-spacing:4px; color:var(--dim);
  display:flex; justify-content:space-between; align-items:center;
}
.viz-live {
  color:var(--cyan); display:flex; align-items:center; gap:6px;
  font-size:10px; animation:blink 2s infinite;
}
.viz-live span { width:6px; height:6px; border-radius:50%; background:var(--cyan); }

/* ── Camera feed area (메인) ─────────────────── */
#cam-area {
  flex: 1;
  position: relative;
  min-height: 300px;
  background: #000;
  overflow: hidden;
  border: 1px solid var(--bd);
}
#viz-feed {
  width: 100%;
  height: 100%;
  object-fit: contain;
  display: block;
}

/* ── Mini spacecraft visualizer (우측 하단 오버레이) ── */
#viz-screen {
  position: absolute;
  bottom: 10px;
  right: 10px;
  width: 300px;
  height: 185px;
  border: 1px solid rgba(0,238,252,0.35);
  background: rgba(4,4,8,0.90);
  background-image:
    linear-gradient(to right, rgba(255,255,255,0.025) 1px, transparent 1px),
    linear-gradient(to bottom, rgba(255,255,255,0.025) 1px, transparent 1px);
  background-size: 20px 20px;
  overflow: hidden;
  z-index: 5;
}
.viz-mini-lbl {
  position: absolute; top: 5px; left: 7px;
  font-size: 7px; font-weight: 700; letter-spacing: 2px;
  color: rgba(0,238,252,0.45); pointer-events: none; z-index: 2; user-select: none;
}
.viz-ch-h { position:absolute; top:50%; left:0; right:0; height:1px; background:rgba(142,145,146,0.1); pointer-events:none; }
.viz-ch-v { position:absolute; left:50%; top:0; bottom:0; width:1px; background:rgba(142,145,146,0.1); pointer-events:none; }
.viz-ring {
  position:absolute; border-radius:50%; pointer-events:none;
  top:50%; left:50%; transform:translate(-50%,-50%);
}

/* HUD overlays — 카메라 위에 표시 */
.hud-tl, .hud-tr {
  position:absolute; top:14px;
  display:flex; flex-direction:column; gap:4px;
  pointer-events:none; user-select:none; z-index:8;
}
.hud-tl { left:14px; }
.hud-tr { right:14px; text-align:right; }
.hud-sub  { font-size:10px; font-weight:700; letter-spacing:4px; color:rgba(160,160,160,0.7);
            text-shadow: 0 1px 4px rgba(0,0,0,0.9); }
.hud-main { font-size:24px; font-weight:700; letter-spacing:2px; transition:color 0.3s;
            text-shadow: 0 2px 8px rgba(0,0,0,0.95); }

/* Laser guide (mini viz 내부) */
#laser { position:absolute; top:50%; left:18%; right:2%; height:1px; background:rgba(0,238,252,0.18); transform:translateY(-50%); pointer-events:none; }

/* ISS block (mini) */
#iss {
  position:absolute; left:4%; top:50%; transform:translateY(-50%);
  width:56px; height:110px;
  border:1px solid rgba(142,145,146,0.35);
  background:rgba(255,255,255,0.01);
  display:flex; align-items:center; justify-content:flex-end; padding-right:5px;
}
.iss-lbl { position:absolute; top:5px; left:5px; font-size:7px; font-weight:700; color:#555; letter-spacing:1px; }
.iss-port {
  width:12px; height:28px;
  background:#121212; border:1px solid #555;
  position:relative; display:flex; align-items:center; justify-content:center;
}
.iss-port::after {
  content:''; position:absolute; right:-6px;
  width:6px; height:2px; background:#aaa; transition:background 0.3s;
}
.iss-port.docked::after { background:var(--green); }

/* CubeSat block (mini) */
#cubesat {
  position:absolute; top:50%; transform:translateY(-50%);
  width:40px; height:40px;
  border:1px solid rgba(142,145,146,0.35);
  background:rgba(255,255,255,0.01);
  display:flex; align-items:center; justify-content:center;
  transition:left 0.35s ease-out;
}
#cubesat.docked { border-color:rgba(0,240,255,0.5); background:rgba(0,240,255,0.02); }
#cubesat-lbl { font-size:6px; color:#888; letter-spacing:1px; text-align:center; }
#cubesat-dist { font-size:7px; color:var(--cyan); margin-top:2px; display:block; }

/* thruster beam (approach indicator) */
#thruster { position:absolute; right:100%; top:50%; transform:translateY(-50%); margin-right:3px; display:none; }
#thruster .beam { width:14px; height:3px; background:var(--cyan); animation:blink 0.4s infinite; border-radius:2px; }
#thruster .tail { width:7px; height:2px; background:rgba(0,238,252,0.5); border-radius:2px; margin-top:1px; }

/* Banner (카메라 위 하단 중앙) */
.viz-banner {
  position:absolute; bottom:200px; left:50%; transform:translateX(-50%);
  padding:8px 20px; border:1px solid; font-size:11px; font-weight:700;
  letter-spacing:2px; display:none; align-items:center; gap:8px;
  white-space:nowrap; z-index:10; user-select:none;
}
.banner-ok   { background:rgba(0,60,20,0.90); border-color:var(--green); color:var(--green); }
.banner-warn { background:rgba(100,60,0,0.90); border-color:var(--amber); color:var(--amber); animation:blink 1s infinite; }
.banner-charge { background:rgba(80,60,0,0.90); border-color:#ffc832; color:#ffc832; }

/* Actuator bar */
.act-wrap { border:1px solid var(--bd); background:var(--bg4); padding:10px 12px; flex-shrink:0; }
.act-row { display:flex; justify-content:space-between; align-items:flex-end; margin-bottom:6px; }
.act-lbl { font-size:9px; font-weight:700; letter-spacing:4px; color:var(--dim); }
.act-pct { font-size:14px; font-weight:700; color:#fff; letter-spacing:2px; }
.act-bar-bg {
  height:20px; background:#131313; border:1px solid var(--bd);
  position:relative; display:flex; align-items:center;
}
.act-bar-fill {
  height:100%; transition:width 0.3s;
  background:linear-gradient(to right, rgba(0,238,252,0.35), var(--cyan));
  position:relative;
}
.act-bar-fill::after { content:''; position:absolute; right:0; top:0; bottom:0; width:4px; background:#fff; }
.act-tick { position:absolute; top:0; bottom:0; width:1px; background:rgba(200,200,200,0.07); }
.act-tick-lbl { position:absolute; bottom:-1px; transform:translateY(100%); font-size:6px; color:#444; font-weight:700; }

/* Manual control display */
.mpc-bar { border:1px solid var(--bd); background:var(--bg4); padding:8px 12px; display:flex; justify-content:space-between; align-items:center; flex-shrink:0; }
.mpc-lbl { font-size:9px; font-weight:700; letter-spacing:3px; color:var(--dim); }
.mpc-val { font-size:13px; font-weight:700; color:var(--cyan); }

/* ── Telemetry Grid RIGHT ────────────────────── */
.tg-section { border:1px solid var(--bd); background:var(--bg4); padding:14px; display:flex; flex-direction:column; gap:10px; }
.tg-hdr { display:flex; justify-content:space-between; align-items:center; border-bottom:1px solid var(--bd); padding-bottom:8px; }
.tg-title { font-size:10px; font-weight:700; letter-spacing:2px; color:#fff; }
.badge { font-size:8px; font-weight:700; letter-spacing:2px; padding:2px 6px; border:1px solid; }
.badge-ok  { color:var(--green); border-color:rgba(0,255,85,0.4); background:rgba(0,255,85,0.08); }
.badge-warn{ color:var(--amber); border-color:rgba(255,183,77,0.4); background:rgba(255,183,77,0.08); }
.badge-dim { color:#555; border-color:#333; background:transparent; }
.step-grid { display:grid; grid-template-columns:1fr 1fr; gap:10px; }
.step-lbl { font-size:8px; font-weight:700; color:#555; letter-spacing:2px; margin-bottom:4px; }
.step-val { font-size:26px; font-weight:700; color:#fff; }
.mdir-bar { border:1px solid var(--bd); background:#131313; padding:8px 10px; display:flex; justify-content:space-between; align-items:center; }
.mdir-sub { font-size:8px; font-weight:700; color:#555; letter-spacing:2px; }
.mdir-val { font-size:10px; font-weight:700; letter-spacing:2px; }
.ar-row { display:flex; justify-content:space-between; align-items:center; margin-bottom:6px; }
.ar-key { font-size:10px; color:#555; letter-spacing:2px; }
.ar-val { font-size:16px; font-weight:700; }
.ar-val.ok   { color:var(--green); }
.ar-val.warn { color:var(--amber); }
.ar-val.dim  { color:var(--dim); }
.abar-bg { height:6px; background:#111; border-radius:3px; position:relative; margin-top:8px; }
.abar-zero { position:absolute; left:50%; top:0; height:100%; width:1px; background:#444; }
.abar-fill { position:absolute; top:0; height:100%; border-radius:3px; transition:all 0.3s; }
.viz-side-hdr { font-size:10px; font-weight:700; letter-spacing:4px; color:var(--dim); }
.cpu-icon { margin-top:auto; opacity:0.07; display:flex; justify-content:flex-end; }

/* ── TELEMETRY tab ───────────────────────────── */
#tab-telemetry { flex-direction:column; }
.tl-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:12px; }
.tl-card { border:1px solid var(--bd); background:#0f0f0f; padding:14px; }
.tl-card .tc-l { font-size:8px; font-weight:700; color:#555; letter-spacing:2px; margin-bottom:4px; text-transform:uppercase; }
.tl-card .tc-v { font-size:22px; font-weight:700; letter-spacing:2px; }
.tl-card .tc-s { font-size:8px; color:#555; margin-top:6px; }
.detail-grid { display:grid; grid-template-columns:1fr 1fr; gap:12px; }
.detail-blk { border:1px solid var(--bd); background:rgba(0,0,0,0.4); padding:16px; }
.detail-blk h3 { font-size:10px; font-weight:700; letter-spacing:2px; color:#fff; border-bottom:1px solid var(--bd); padding-bottom:6px; margin-bottom:10px; }
.drow { display:flex; justify-content:space-between; border-bottom:1px solid #0e0e0e; padding:6px 0; font-size:10px; }
.drow:last-child { border-bottom:none; }
.dk { color:#555; }
.dv { font-weight:700; color:#fff; }

/* ── POWER tab ───────────────────────────────── */
#tab-power { flex-direction:column; }
.pw-grid { display:grid; grid-template-columns:repeat(2,1fr); gap:12px; }
.pw-card { border:1px solid var(--bd); background:rgba(0,0,0,0.4); padding:16px; }
.pw-card h3 { font-size:10px; font-weight:700; letter-spacing:2px; color:#fff; margin-bottom:12px; }
.mag-big { font-size:48px; font-weight:700; letter-spacing:4px; text-align:center; padding:16px 0; transition:color 0.3s; }
.servo-display { text-align:center; padding:8px 0; }
.servo-val { font-size:36px; font-weight:700; color:var(--cyan); }
.servo-lbl { font-size:10px; color:#555; margin-top:4px; letter-spacing:2px; }
.sm-grid { display:grid; grid-template-columns:repeat(6,1fr); gap:8px; margin-top:8px; }
.sm-card {
  text-align:center; padding:12px 4px;
  border:1px solid var(--bd); background:#0a0a0a;
  transition:border-color 0.3s;
}
.sm-idx { font-size:8px; color:#555; letter-spacing:1px; margin-bottom:6px; }
.sm-name { font-size:8px; font-weight:700; letter-spacing:1px; }
.sm-act { font-size:7px; color:#333; margin-top:6px; line-height:1.5; }

/* ── PAYLOAD tab ─────────────────────────────── */
#tab-payload { flex-direction:column; }
#payload-feed { width:100%; max-height:460px; object-fit:contain; border:1px solid var(--bd); display:block; }
.pl-meta { display:grid; grid-template-columns:repeat(4,1fr); gap:10px; margin-top:12px; }
.pl-card { border:1px solid var(--bd); background:#0f0f0f; padding:10px; }
.pl-l { font-size:8px; font-weight:700; color:#555; letter-spacing:2px; margin-bottom:4px; }
.pl-v { font-size:16px; font-weight:700; }

/* State colors */
.sc-PRE_DOCKING  { color:#a0a0a0; }
.sc-TARGET_LOCK  { color:var(--cyan); }
.sc-SOFT_CAPTURE { color:var(--green); }
.sc-HARD_LOCK    { color:#ff9800; }
.sc-DOCKED       { color:var(--green); }
.sc-CHARGING     { color:#ffc832; }
</style>
</head>
<body>

<!-- ═══ TOP BAR ══════════════════════════════════════════ -->
<header id="topbar">
  <div style="display:flex;align-items:center;gap:16px;">
    <span class="sys-name">CUBESAT_OS_v2.4</span>
    <div class="status-pill" id="status-pill">
      <div class="status-dot" id="status-dot" style="background:#555;"></div>
      <span id="status-text">MISSION_STATUS: STANDBY</span>
    </div>
  </div>
  <div class="topbar-right">
    <span style="font-size:10px;font-weight:700;letter-spacing:2px;color:#555;">SIMULATION_MODE &nbsp; <span style="color:#333;">OFF</span></span>
    <span id="clock"></span>
  </div>
</header>

<!-- ═══ BODY ══════════════════════════════════════════════ -->
<div id="body">

  <!-- Sidebar -->
  <aside id="sidebar">
    <div class="sb-header">
      <div class="sb-mc">MISSION_CONTROL</div>
      <div class="sb-orbit">CAS500-2_ORBIT</div>
    </div>
    <nav>
      <button class="nav-btn active" data-tab="visualizer" onclick="switchTab('visualizer')">
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <circle cx="12" cy="12" r="3"/>
          <path d="M12 1v4M12 19v4M4.22 4.22l2.83 2.83M16.95 16.95l2.83 2.83M1 12h4M19 12h4M4.22 19.78l2.83-2.83M16.95 7.05l2.83-2.83"/>
        </svg>
        VISUALIZER
      </button>
      <button class="nav-btn" data-tab="telemetry" onclick="switchTab('telemetry')">
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <rect x="5" y="2" width="14" height="20" rx="2"/>
          <path d="M9 7h6M9 11h6M9 15h4"/>
        </svg>
        TELEMETRY
      </button>
      <button class="nav-btn" data-tab="power" onclick="switchTab('power')">
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <path d="M13 2L3 14h9l-1 8 10-12h-9l1-8z"/>
        </svg>
        POWER
      </button>
      <button class="nav-btn" data-tab="payload" onclick="switchTab('payload')">
        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <rect x="2" y="7" width="20" height="15" rx="2"/>
          <polyline points="17 2 12 7 7 2"/>
        </svg>
        PAYLOAD
      </button>
    </nav>
    <div class="sb-footer"><span>SECURE ENCRYPTED COMM-LINE</span></div>
  </aside>

  <!-- Content -->
  <div id="content">

    <!-- ═══ VISUALIZER TAB ═══════════════════════════════ -->
    <div id="tab-visualizer" class="panel active">

      <!-- Left: Dynamics Visualizer -->
      <div id="viz-main">
        <div class="viz-hdr">
          <span>// DYNAMICS_VISUALIZER</span>
          <span class="viz-live"><span></span>CAM_FEED_01</span>
        </div>

        <!-- 카메라 피드 (메인) + 오버레이 -->
        <div id="cam-area">
          <!-- 실시간 카메라 스트림 -->
          <img id="viz-feed" src="/video_feed" alt="Camera Feed">

          <!-- HUD: 거리 (좌상단 카메라 위) -->
          <div class="hud-tl">
            <div class="hud-sub">RANGE_TO_TARGET</div>
            <div class="hud-main" id="hud-dist" style="color:#fff;">DIST: ---</div>
          </div>

          <!-- HUD: 접근속도 (우상단 카메라 위) -->
          <div class="hud-tr">
            <div class="hud-sub">RADIAL_APPROACH_RATE</div>
            <div class="hud-main" id="hud-rate" style="color:#fff;">RATE: +0.0mm/s</div>
          </div>

          <!-- 상태 배너 (카메라 위 하단 중앙) -->
          <div class="viz-banner" id="viz-banner"></div>

          <!-- Mini 궤도 시각화 (우측 하단 오버레이) -->
          <div id="viz-screen">
            <div class="viz-mini-lbl">// DYNAMICS_VISUALIZER</div>
            <div class="viz-ch-h"></div>
            <div class="viz-ch-v"></div>
            <div class="viz-ring" style="width:56px;height:56px;border:1px dashed rgba(142,145,146,0.18);"></div>
            <div class="viz-ring" style="width:120px;height:120px;border:1px solid rgba(142,145,146,0.1);"></div>

            <div id="laser"></div>

            <div id="iss">
              <div class="iss-lbl">ISS</div>
              <div class="iss-port" id="iss-port"></div>
            </div>

            <div id="cubesat" style="left:80%;">
              <div id="thruster">
                <div class="beam"></div>
                <div class="tail"></div>
              </div>
              <div id="cubesat-lbl">
                COS<span id="cubesat-dist">---</span>
              </div>
            </div>
          </div><!-- /viz-screen -->

        </div><!-- /cam-area -->

        <!-- Actuator alignment bar -->
        <div class="act-wrap">
          <div class="act-row">
            <span class="act-lbl">ACTUATOR_ALIGNMENT_POSITION</span>
            <span class="act-pct" id="act-pct">0.0%</span>
          </div>
          <div class="act-bar-bg">
            <div class="act-bar-fill" id="act-fill" style="width:0%;"></div>
            <div class="act-tick" style="left:25%;"><span class="act-tick-lbl">25</span></div>
            <div class="act-tick" style="left:50%;"><span class="act-tick-lbl" style="color:rgba(0,238,252,0.4);">50</span></div>
            <div class="act-tick" style="left:75%;"><span class="act-tick-lbl">75</span></div>
          </div>
        </div>

        <!-- Manual position control (display only) -->
        <div class="mpc-bar">
          <span class="mpc-lbl">MANUAL_POSITION_CONTROL</span>
          <span class="mpc-val" id="mpc-val">+0.00</span>
        </div>
      </div><!-- /viz-main -->

      <!-- Right: Telemetry Grid -->
      <div id="viz-side">
        <div class="viz-side-hdr">// TELEMETRY_GRID</div>

        <!-- MOTOR_STATS -->
        <div class="tg-section">
          <div class="tg-hdr">
            <span class="tg-title">MOTOR_STATS</span>
            <span class="badge badge-dim" id="motor-badge">STANDBY</span>
          </div>
          <div class="step-grid">
            <div>
              <div class="step-lbl">CURRENT_STEP</div>
              <div class="step-val" id="tg-cur">0</div>
            </div>
            <div>
              <div class="step-lbl">TARGET_STEP</div>
              <div class="step-val" style="color:#888;" id="tg-tgt">0</div>
            </div>
          </div>
          <div class="mdir-bar">
            <span class="mdir-sub">MOTOR_STEERING</span>
            <span class="mdir-val" id="motor-dir" style="color:#555;">&#9632; STANDBY</span>
          </div>
        </div>

        <!-- ALIGNMENT_STATUS (ArUco) -->
        <div class="tg-section">
          <div class="tg-hdr">
            <span class="tg-title">ALIGNMENT_STATUS</span>
            <span class="badge badge-dim" id="aruco-badge">NO LOCK</span>
          </div>
          <div class="ar-row">
            <span class="ar-key">MARKER</span>
            <span class="ar-val dim" id="tg-marker">NO LOCK</span>
          </div>
          <div class="ar-row">
            <span class="ar-key">ROLL</span>
            <span class="ar-val dim" id="tg-angle">---</span>
          </div>
          <div class="ar-row">
            <span class="ar-key">dX</span>
            <span class="ar-val dim" id="tg-dx">---</span>
          </div>
          <div class="ar-row">
            <span class="ar-key">dY</span>
            <span class="ar-val dim" id="tg-dy">---</span>
          </div>
          <div class="abar-bg">
            <div class="abar-zero"></div>
            <div class="abar-fill" id="tg-abar" style="left:50%;width:0%;background:var(--green);"></div>
          </div>
        </div>

        <!-- Decorative icon -->
        <div class="cpu-icon">
          <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.2">
            <rect x="7" y="7" width="10" height="10" rx="1"/>
            <path d="M9 7V4M12 7V4M15 7V4M9 17v3M12 17v3M15 17v3M7 9H4M7 12H4M7 15H4M17 9h3M17 12h3M17 15h3"/>
          </svg>
        </div>
      </div><!-- /viz-side -->

    </div><!-- /tab-visualizer -->

    <!-- ═══ TELEMETRY TAB ════════════════════════════════ -->
    <div id="tab-telemetry" class="panel" style="flex-direction:column;">
      <div class="panel-hdr">
        <h2>// SENSOR &amp; STATE TELEMETRY LOGS</h2>
        <p>Physical telemetry framework &amp; sensor evaluation matrices.</p>
      </div>

      <div class="tl-grid">
        <div class="tl-card">
          <div class="tc-l">RANGE TO TARGET</div>
          <div class="tc-v" id="tl-dist" style="color:var(--cyan);">--- mm</div>
          <div class="tc-s">SOFT CAPTURE ≤ 300mm</div>
        </div>
        <div class="tl-card">
          <div class="tc-l">RADIAL APPROACH RATE</div>
          <div class="tc-v" id="tl-rate" style="color:#f06292;">+0.0 mm/s</div>
          <div class="tc-s">COMPUTED FROM TOF DELTA</div>
        </div>
        <div class="tl-card">
          <div class="tc-l">STEPPER POSITION</div>
          <div class="tc-v" id="tl-steps" style="color:#fff;">0 / 0</div>
          <div class="tc-s">TARGET: 3000 STEPS MAX</div>
        </div>
        <div class="tl-card">
          <div class="tc-l">DOCKING STATE</div>
          <div class="tc-v sc-PRE_DOCKING" id="tl-state">PRE_DOCKING</div>
          <div class="tc-s">6-STATE MACHINE</div>
        </div>
        <div class="tl-card">
          <div class="tc-l">MARKER ROLL ANGLE</div>
          <div class="tc-v" id="tl-angle" style="color:#ce93d8;">--- °</div>
          <div class="tc-s">OK: |ANGLE| &lt; 5 deg</div>
        </div>
        <div class="tl-card">
          <div class="tc-l">MG992 SERVO ANGLE</div>
          <div class="tc-v" id="tl-servo" style="color:var(--amber);">0 °</div>
          <div class="tc-s">CHARGING POSITION: 45 deg</div>
        </div>
      </div>

      <div class="detail-grid">
        <div class="detail-blk">
          <h3>APPROACH VECTOR CALIBRATOR</h3>
          <div class="drow"><span class="dk">ARUCO MARKER</span><span class="dv" id="dl-marker">NO LOCK</span></div>
          <div class="drow"><span class="dk">OFFSET dX</span><span class="dv" id="dl-dx">---</span></div>
          <div class="drow"><span class="dk">OFFSET dY</span><span class="dv" id="dl-dy">---</span></div>
          <div class="drow"><span class="dk">MOTOR DIRECTION</span><span class="dv" id="dl-dir">STANDBY</span></div>
          <div class="drow"><span class="dk">STEP / TARGET</span><span class="dv" id="dl-step">0 / 0</span></div>
          <div class="drow"><span class="dk">MISSION TIME</span><span class="dv" id="dl-mtime">T+ 00:00:00</span></div>
        </div>
        <div class="detail-blk">
          <h3>// DOCKING SAFETY MATRIX</h3>
          <div class="drow"><span class="dk">ELECTROMAGNET</span><span class="dv" id="dl-mag" style="color:#555;">OFF</span></div>
          <div class="drow"><span class="dk">SOFT CAPTURE RANGE</span><span class="dv">≤ 300 mm / 3 s</span></div>
          <div class="drow"><span class="dk">HARD LOCK RANGE</span><span class="dv">≤ 15 mm / 5 s</span></div>
          <div class="drow"><span class="dk">MARKER LOCK TIME</span><span class="dv">3 s CONTINUOUS</span></div>
          <div class="drow"><span class="dk">DOCKED → CHARGING</span><span class="dv">2 s DELAY</span></div>
          <div style="margin-top:12px;padding:8px;background:rgba(113,88,0,0.1);border:1px solid rgba(255,183,77,0.25);font-size:9px;color:var(--amber);">
            <strong>CAUTION:</strong> ArUco marker must be visible for 3s continuously before TARGET_LOCK transition activates.
          </div>
        </div>
      </div>
    </div><!-- /tab-telemetry -->

    <!-- ═══ POWER TAB ════════════════════════════════════ -->
    <div id="tab-power" class="panel" style="flex-direction:column;">
      <div class="panel-hdr">
        <h2>// ONBOARD POWER &amp; ACTUATOR DIAGNOSTICS</h2>
        <p>Electromagnet, stepper driver, and MG992 charging servo status.</p>
      </div>

      <div class="pw-grid">
        <div class="pw-card">
          <h3>ELECTROMAGNET STATUS</h3>
          <div class="mag-big" id="mag-big" style="color:#e74c3c;">OFF</div>
          <div style="font-size:9px;color:#555;letter-spacing:2px;text-align:center;line-height:2;">
            ON: SOFT_CAPTURE &rarr; HARD_LOCK<br>OFF: ALL OTHER STATES
          </div>
        </div>
        <div class="pw-card">
          <h3>MG992 CHARGING SERVO</h3>
          <div class="servo-display">
            <div class="servo-val" id="servo-big">0 °</div>
            <div class="servo-lbl">CURRENT ANGLE</div>
          </div>
          <div style="margin-top:12px;font-size:9px;color:#555;letter-spacing:2px;text-align:center;">
            STOWED: 0 deg &nbsp;|&nbsp; CHARGING: 45 deg
          </div>
        </div>
        <div class="pw-card" style="grid-column:span 2;">
          <h3>STATE MACHINE OVERVIEW</h3>
          <div class="sm-grid" id="sm-grid">
            <div class="sm-card" id="sm0">
              <div class="sm-idx">STATE 1</div>
              <div class="sm-name sc-PRE_DOCKING">PRE_DOCKING</div>
              <div class="sm-act">MAG: OFF<br>MOT: 0</div>
            </div>
            <div class="sm-card" id="sm1">
              <div class="sm-idx">STATE 2</div>
              <div class="sm-name sc-TARGET_LOCK">TARGET_LOCK</div>
              <div class="sm-act">MAG: OFF<br>MOT: &rarr;3000</div>
            </div>
            <div class="sm-card" id="sm2">
              <div class="sm-idx">STATE 3</div>
              <div class="sm-name sc-SOFT_CAPTURE">SOFT_CAPTURE</div>
              <div class="sm-act">MAG: ON<br>MOT: 3000</div>
            </div>
            <div class="sm-card" id="sm3">
              <div class="sm-idx">STATE 4</div>
              <div class="sm-name sc-HARD_LOCK">HARD_LOCK</div>
              <div class="sm-act">MAG: ON<br>MOT: &rarr;0</div>
            </div>
            <div class="sm-card" id="sm4">
              <div class="sm-idx">STATE 5</div>
              <div class="sm-name sc-DOCKED">DOCKED</div>
              <div class="sm-act">MAG: OFF<br>MOT: 0</div>
            </div>
            <div class="sm-card" id="sm5">
              <div class="sm-idx">STATE 6</div>
              <div class="sm-name sc-CHARGING">CHARGING</div>
              <div class="sm-act">MAG: OFF<br>SVO: 45 deg</div>
            </div>
          </div>
        </div>
      </div>
    </div><!-- /tab-power -->

    <!-- ═══ PAYLOAD TAB ══════════════════════════════════ -->
    <div id="tab-payload" class="panel" style="flex-direction:column;">
      <div class="panel-hdr">
        <h2>// CAMERA FEED &amp; OPTICAL SENSOR</h2>
        <p>Live camera stream with ArUco marker detection overlay.</p>
      </div>
      <img id="payload-feed" src="/video_feed" alt="Camera Feed">
      <div class="pl-meta">
        <div class="pl-card"><div class="pl-l">MARKER DETECT</div><div class="pl-v" id="pl-marker" style="color:var(--dim);">NO LOCK</div></div>
        <div class="pl-card"><div class="pl-l">ROLL ANGLE</div><div class="pl-v" id="pl-angle" style="color:#ce93d8;">---</div></div>
        <div class="pl-card"><div class="pl-l">OFFSET dX</div><div class="pl-v" id="pl-dx" style="color:var(--dim);">---</div></div>
        <div class="pl-card"><div class="pl-l">OFFSET dY</div><div class="pl-v" id="pl-dy" style="color:var(--dim);">---</div></div>
      </div>
    </div><!-- /tab-payload -->

  </div><!-- /content -->
</div><!-- /body -->

<!-- ═══ FOOTER ════════════════════════════════════════════ -->
<footer id="footer">
  <div>
    <div class="f-label">MISSION TIME</div>
    <div class="f-val" id="f-mtime">T+ 00:00:00</div>
  </div>

  <div class="seq-wrap">
    <div class="f-label">SEQUENCE</div>
    <div class="seq-dots">
      <div class="seq-line-bg"></div>
      <div class="seq-step"><div class="seq-dot current" id="sd0"></div><span class="seq-lbl current" id="sl0">PRE</span></div>
      <div class="seq-step"><div class="seq-dot" id="sd1"></div><span class="seq-lbl" id="sl1">T.LOCK</span></div>
      <div class="seq-step"><div class="seq-dot" id="sd2"></div><span class="seq-lbl" id="sl2">S.CAP</span></div>
      <div class="seq-step"><div class="seq-dot" id="sd3"></div><span class="seq-lbl" id="sl3">H.LOCK</span></div>
      <div class="seq-step"><div class="seq-dot" id="sd4"></div><span class="seq-lbl" id="sl4">DOCKED</span></div>
      <div class="seq-step"><div class="seq-dot" id="sd5"></div><span class="seq-lbl" id="sl5">CHARGING</span></div>
    </div>
  </div>

  <button id="reset-btn" onclick="doReset()">&#x27F3; RESET</button>
</footer>

<script>
// ─────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────
const STATES = ['PRE_DOCKING','TARGET_LOCK','SOFT_CAPTURE','HARD_LOCK','DOCKED','CHARGING'];

const STATE_COLORS = {
  PRE_DOCKING:  '#a0a0a0',
  TARGET_LOCK:  '#00eefc',
  SOFT_CAPTURE: '#00FF55',
  HARD_LOCK:    '#ff9800',
  DOCKED:       '#00FF55',
  CHARGING:     '#ffc832',
};

const STATUS_CFG = {
  PRE_DOCKING:  { dot:'#555',   border:'rgba(100,100,100,0.3)', bg:'rgba(100,100,100,0.08)', label:'MISSION_STATUS: STANDBY' },
  TARGET_LOCK:  { dot:'#00eefc',border:'rgba(0,238,252,0.3)',   bg:'rgba(0,238,252,0.08)',   label:'MISSION_STATUS: TARGET_LOCKED' },
  SOFT_CAPTURE: { dot:'#00FF55',border:'rgba(0,255,85,0.3)',    bg:'rgba(0,255,85,0.08)',    label:'STATUS: SOFT_CAPTURE ENGAGED' },
  HARD_LOCK:    { dot:'#ff9800',border:'rgba(255,152,0,0.3)',   bg:'rgba(255,152,0,0.08)',   label:'STATUS: ALIGNMENT LOCK ALERT' },
  DOCKED:       { dot:'#00FF55',border:'rgba(0,255,85,0.3)',    bg:'rgba(0,255,85,0.08)',    label:'MISSION_STATUS: DOCKED' },
  CHARGING:     { dot:'#ffc832',border:'rgba(255,200,50,0.3)',  bg:'rgba(255,200,50,0.08)',  label:'MISSION_STATUS: CHARGING ACTIVE' },
};

// Distance → left% mapping for CubeSat position
const DIST_MAX = 500;  // mm — clamp at 500mm for visualization
const POS_NEAR = 35;   // % — position when distance ≈ 0 (just past ISS)
const POS_FAR  = 80;   // % — position when distance ≥ 500mm

// ─────────────────────────────────────────────
// Velocity tracking
// ─────────────────────────────────────────────
let prevDist = null;
let prevTime = null;
let velocity = 0; // mm/s (negative = approaching)

// ─────────────────────────────────────────────
// Tab switching
// ─────────────────────────────────────────────
function switchTab(tab) {
  document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.nav-btn').forEach(b => b.classList.remove('active'));
  document.getElementById('tab-' + tab).classList.add('active');
  document.querySelector('.nav-btn[data-tab="' + tab + '"]').classList.add('active');
}

// ─────────────────────────────────────────────
// Main update loop (polls /api/status every 500ms)
// ─────────────────────────────────────────────
async function update() {
  try {
    const d = await fetch('/api/status').then(r => r.json());
    const now = Date.now();

    // Compute velocity from consecutive distance readings
    if (prevDist !== null && prevTime !== null) {
      const dt = (now - prevTime) / 1000;
      if (dt > 0.05) velocity = (d.distance_mm - prevDist) / dt;
    }
    prevDist = d.distance_mm;
    prevTime = now;

    const dist   = d.distance_mm;
    const state  = d.state;
    const magnet = d.magnet;
    const steps  = d.motor_steps;
    const tgt    = d.motor_target;
    const servo  = d.servo_angle;
    const mtime  = d.mission_time;
    const marker = d.marker_detected;
    const ang    = d.marker_angle;
    const dx     = d.marker_offset_x;
    const dy     = d.marker_offset_y;

    const isDocked   = (state === 'DOCKED' || state === 'CHARGING');
    const isCharging = (state === 'CHARGING');
    const stateIdx   = STATES.indexOf(state);
    const stateColor = STATE_COLORS[state] || '#a0a0a0';
    const sc         = STATUS_CFG[state] || STATUS_CFG.PRE_DOCKING;

    // ── TopBar ────────────────────────────────
    const pill = document.getElementById('status-pill');
    pill.style.borderColor = sc.border;
    pill.style.background  = sc.bg;
    pill.style.color       = sc.dot;
    document.getElementById('status-dot').style.background = sc.dot;
    document.getElementById('status-text').textContent = sc.label;

    // ── Mission time ──────────────────────────
    document.getElementById('f-mtime').textContent  = mtime;
    document.getElementById('dl-mtime').textContent = mtime;

    // ── CubeSat position ──────────────────────
    // Map distance 0–500mm → POS_NEAR–POS_FAR %
    const clampedDist = dist >= 0 ? Math.max(0, Math.min(DIST_MAX, dist)) : DIST_MAX;
    const leftPct = POS_NEAR + (clampedDist / DIST_MAX) * (POS_FAR - POS_NEAR);
    document.getElementById('cubesat').style.left = leftPct + '%';
    document.getElementById('cubesat').classList.toggle('docked', isDocked);
    document.getElementById('iss-port').classList.toggle('docked', isDocked);

    // CubeSat label
    const distLbl = dist >= 0 ? (isDocked ? 'LOCK' : Math.round(dist) + 'mm') : '---';
    document.getElementById('cubesat-dist').textContent = distLbl;

    // Thruster beam (show when approaching: velocity < -2mm/s)
    document.getElementById('thruster').style.display = (velocity < -2 && !isDocked) ? 'block' : 'none';

    // ── HUD Overlays ──────────────────────────
    const hudDist = document.getElementById('hud-dist');
    hudDist.textContent = 'DIST: ' + (dist >= 0 ? Math.round(dist) + 'mm' : '---');
    hudDist.style.color = isDocked ? '#00eefc' : '#fff';

    const rateStr = (velocity >= 0 ? '+' : '') + velocity.toFixed(1) + 'mm/s';
    const hudRate = document.getElementById('hud-rate');
    hudRate.textContent = 'RATE: ' + rateStr;
    const isHighSpeed = (dist >= 0 && dist < 150 && velocity < -20);
    hudRate.style.color = isHighSpeed ? '#f44336' : (Math.abs(velocity) > 2 ? '#f06292' : '#fff');

    // ── Banner ────────────────────────────────
    const banner = document.getElementById('viz-banner');
    if (isCharging) {
      banner.className = 'viz-banner banner-charge';
      banner.textContent = '&#x26A1; CHARGING MECHANISM ACTIVE';
      banner.style.display = 'flex';
    } else if (isDocked) {
      banner.className = 'viz-banner banner-ok';
      banner.textContent = '&#x2713; DOCKING MECHANISM LOCK SECURED';
      banner.style.display = 'flex';
    } else if (isHighSpeed) {
      banner.className = 'viz-banner banner-warn';
      banner.textContent = '&#x26A0; APPROACH VELOCITY WARNING';
      banner.style.display = 'flex';
    } else {
      banner.style.display = 'none';
    }

    // ── Actuator bar (motor_steps / 3000) ─────
    const maxSteps = tgt > 0 ? tgt : 3000;
    const actPct = Math.min(100, steps / maxSteps * 100);
    document.getElementById('act-pct').textContent = actPct.toFixed(1) + '%';
    document.getElementById('act-fill').style.width = actPct + '%';

    // Manual control display (velocity-based)
    document.getElementById('mpc-val').textContent = (velocity >= 0 ? '+' : '') + (velocity / 100).toFixed(2);

    // ── Telemetry Grid: Motor ─────────────────
    document.getElementById('tg-cur').textContent = steps;
    document.getElementById('tg-tgt').textContent = tgt;

    const motorBadge = document.getElementById('motor-badge');
    if (state === 'TARGET_LOCK' || state === 'SOFT_CAPTURE' || state === 'HARD_LOCK') {
      motorBadge.className = 'badge badge-ok';
      motorBadge.textContent = 'ACTIVE';
    } else {
      motorBadge.className = 'badge badge-dim';
      motorBadge.textContent = 'STANDBY';
    }

    let dirTxt = '&#9632; STANDBY';
    let dirColor = '#555';
    if (steps < tgt) { dirTxt = '&rarr; FORWARD'; dirColor = '#00eefc'; }
    else if (steps > tgt) { dirTxt = '&larr; REVERSE'; dirColor = '#00eefc'; }
    const mdir = document.getElementById('motor-dir');
    mdir.innerHTML = dirTxt;
    mdir.style.color = dirColor;

    // ── Telemetry Grid: ArUco ─────────────────
    const arucoBadge = document.getElementById('aruco-badge');
    const tgMarker = document.getElementById('tg-marker');
    const tgAngle  = document.getElementById('tg-angle');
    const tgDx     = document.getElementById('tg-dx');
    const tgDy     = document.getElementById('tg-dy');
    const abar     = document.getElementById('tg-abar');

    if (marker) {
      arucoBadge.className = 'badge badge-ok';
      arucoBadge.textContent = 'LOCKED';
      tgMarker.textContent = 'LOCKED'; tgMarker.className = 'ar-val ok';
      const angOk = Math.abs(ang) < 5;
      const dxOk  = Math.abs(dx)  < 20;
      const dyOk  = Math.abs(dy)  < 20;
      tgAngle.textContent = (ang >= 0 ? '+' : '') + ang.toFixed(1) + 'deg';
      tgAngle.className   = 'ar-val ' + (angOk ? 'ok' : 'warn');
      tgDx.textContent    = (dx  >= 0 ? '+' : '') + dx.toFixed(0) + 'px';
      tgDx.className      = 'ar-val ' + (dxOk ? 'ok' : 'warn');
      tgDy.textContent    = (dy  >= 0 ? '+' : '') + dy.toFixed(0) + 'px';
      tgDy.className      = 'ar-val ' + (dyOk ? 'ok' : 'warn');
      const clamp = Math.max(-45, Math.min(45, ang));
      const bp    = (clamp + 45) / 90 * 100;
      abar.style.left       = Math.min(50, bp) + '%';
      abar.style.width      = Math.abs(bp - 50) + '%';
      abar.style.background = angOk ? 'var(--green)' : 'var(--amber)';
    } else {
      arucoBadge.className = 'badge badge-dim';
      arucoBadge.textContent = 'NO LOCK';
      ['tg-marker','tg-angle','tg-dx','tg-dy'].forEach(id => {
        const el = document.getElementById(id);
        el.textContent = id === 'tg-marker' ? 'NO LOCK' : '---';
        el.className = 'ar-val dim';
      });
      abar.style.width = '0%';
    }

    // ── Sequence Dots ─────────────────────────
    for (let i = 0; i < 6; i++) {
      const dot = document.getElementById('sd' + i);
      const lbl = document.getElementById('sl' + i);
      dot.className = 'seq-dot' + (i < stateIdx ? ' done' : i === stateIdx ? ' current' : '');
      lbl.className = 'seq-lbl' + (i < stateIdx ? ' done' : i === stateIdx ? ' current' : '');
    }

    // ── Telemetry Tab ─────────────────────────
    const tlDist = document.getElementById('tl-dist');
    tlDist.textContent = dist >= 0 ? Math.round(dist) + ' mm' : '--- mm';
    tlDist.style.color = (dist >= 0 && dist <= 300) ? 'var(--amber)' : 'var(--cyan)';

    document.getElementById('tl-rate').textContent = rateStr;
    document.getElementById('tl-steps').textContent = steps + ' / ' + tgt;

    const tlState = document.getElementById('tl-state');
    tlState.textContent = state;
    tlState.style.color = stateColor;

    document.getElementById('tl-angle').textContent = marker ? (ang >= 0 ? '+' : '') + ang.toFixed(1) + ' deg' : '--- deg';
    document.getElementById('tl-servo').textContent = servo + ' deg';

    document.getElementById('dl-marker').textContent = marker ? 'LOCKED' : 'NO LOCK';
    document.getElementById('dl-marker').style.color = marker ? 'var(--green)' : '#555';
    document.getElementById('dl-dx').textContent = marker ? (dx  >= 0 ? '+' : '') + dx.toFixed(0) + 'px' : '---';
    document.getElementById('dl-dy').textContent = marker ? (dy  >= 0 ? '+' : '') + dy.toFixed(0) + 'px' : '---';
    document.getElementById('dl-dir').innerHTML = dirTxt;
    document.getElementById('dl-step').textContent = steps + ' / ' + tgt;

    const dlMag = document.getElementById('dl-mag');
    dlMag.textContent = magnet ? 'ON' : 'OFF';
    dlMag.style.color = magnet ? 'var(--green)' : '#555';

    // ── Power Tab ─────────────────────────────
    const magBig = document.getElementById('mag-big');
    magBig.textContent = magnet ? 'ON' : 'OFF';
    magBig.style.color = magnet ? 'var(--green)' : '#e74c3c';
    document.getElementById('servo-big').textContent = servo + ' deg';

    // Highlight active state card in SM overview
    for (let i = 0; i < 6; i++) {
      const card = document.getElementById('sm' + i);
      card.style.borderColor = (i === stateIdx) ? stateColor : 'var(--bd)';
      card.style.background  = (i === stateIdx) ? 'rgba(255,255,255,0.04)' : '#0a0a0a';
    }

    // ── Payload Tab ───────────────────────────
    const plMarker = document.getElementById('pl-marker');
    plMarker.textContent = marker ? 'LOCKED' : 'NO LOCK';
    plMarker.style.color = marker ? 'var(--green)' : 'var(--dim)';
    document.getElementById('pl-angle').textContent = marker ? (ang >= 0 ? '+' : '') + ang.toFixed(1) + ' deg' : '---';
    document.getElementById('pl-dx').textContent    = marker ? (dx  >= 0 ? '+' : '') + dx.toFixed(0) + 'px' : '---';
    document.getElementById('pl-dy').textContent    = marker ? (dy  >= 0 ? '+' : '') + dy.toFixed(0) + 'px' : '---';

  } catch(e) {
    console.warn('[UI] status fetch failed:', e);
  }
}

// ─────────────────────────────────────────────
// Reset button
// ─────────────────────────────────────────────
async function doReset() {
  const btn = document.getElementById('reset-btn');
  btn.textContent = '...';
  btn.disabled = true;
  try {
    await fetch('/api/reset', { method: 'POST' });
    prevDist = null; prevTime = null; velocity = 0;
    await update();
  } catch(e) {}
  setTimeout(() => { btn.textContent = '&#x27F3; RESET'; btn.disabled = false; }, 900);
}

// ─────────────────────────────────────────────
// Clock & polling
// ─────────────────────────────────────────────
setInterval(() => {
  document.getElementById('clock').textContent = new Date().toLocaleString('ko-KR');
}, 1000);

setInterval(update, 500);
update();
</script>
</body>
</html>"""

# ════════════════════════════════════════════════════════
# ── 메인 실행 ───────────────────────────────────────────
# ════════════════════════════════════════════════════════
if __name__ == '__main__':
    threading.Thread(target=tof_thread,           daemon=True, name="ToF").start()
    threading.Thread(target=camera_thread,        daemon=True, name="Camera").start()
    threading.Thread(target=motor_thread,         daemon=True, name="Motor").start()
    threading.Thread(target=state_machine_thread, daemon=True, name="StateMachine").start()

    print("=" * 52)
    print("  CubeSat Docking + Charging System  |  main_ver1.py")
    print("  6-State: PRE → T.LOCK → S.CAP → H.LOCK → DOCKED → CHARGING")
    print(f"  웹 UI : http://pi.local:5000")
    print("=" * 52)

    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        _set_magnet(False)
        _set_motor_target(0)
        _set_servo(SERVO_STOWED_ANGLE)
        if GPIO_OK:
            _pwm.stop()
            if _servo_pwm is not None:
                _servo_pwm.stop()
            GPIO.cleanup()
        print("\n[SYS] 종료 완료")
