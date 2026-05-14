"""
main_ver1.py
============
CubeSat 도킹 모듈 - 2차 시연 메인 스크립트

5-State Machine:
  State 1  PRE_DOCKING  : 초기 대기 — 전자석 OFF, 모터 step 0
  State 2  TARGET_LOCK  : ArUco 마커 감지 — 모터 step 0→3000
  State 3  SOFT_CAPTURE : ToF ≤ 300mm (3s) — 전자석 ON, 모터 step 3000 유지
  State 4  HARD_LOCK    : ToF ≤ 15mm  (5s) — 전자석 ON,  모터 step 3000→0
  State 5  DOCKED       : 모터 step 0 도달 후 2s — 전자석 OFF, step 0 유지

State 전이:
  PRE_DOCKING  → TARGET_LOCK  : ArUco 마커 감지
  TARGET_LOCK  → PRE_DOCKING  : 마커 1.5s 소실 (미진입 시에만)
  TARGET_LOCK  → SOFT_CAPTURE : ToF ≤ 300mm 3s 지속
  SOFT_CAPTURE → HARD_LOCK    : ToF ≤ 15mm  5s 지속
  HARD_LOCK    → DOCKED       : 모터 step 0 도달 후 2s

실행: python3 main_ver1.py
접속: http://<pi_ip>:5000
"""

import time
import math
import threading
import config
from flask import Flask, Response, jsonify

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
        #   조건: 없음 (평상시)
        #   행동: 전자석 OFF, 모터 step 0
        #   전이: ArUco 마커 3s 연속 감지 → TARGET_LOCK
        # ═══════════════════════════════════════════════
        if state == "PRE_DOCKING":
            _set_magnet(False)
            _set_motor_target(0)
            dist_300_start    = None
            dist_lock_start   = None
            motor_zero_done_t = None

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
        #   조건: ArUco 마커 3s 연속 감지
        #   행동: 전자석 OFF, 모터 step 0→3000
        #   전이: ToF ≤ 300mm 3s 지속 → SOFT_CAPTURE
        #   ※ 마커 소실돼도 뒤로 돌아가지 않음
        # ═══════════════════════════════════════════════
        elif state == "TARGET_LOCK":
            # 모터는 진입 시 한 번만 설정 — 이미 3000이면 무시됨
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
        #   조건: ToF ≤ 300mm (3s)
        #   행동: 전자석 ON, 모터 step 3000 유지
        #   전이: ToF ≤ 50mm 5s 지속 → HARD_LOCK
        # ═══════════════════════════════════════════════
        elif state == "SOFT_CAPTURE":
            _set_magnet(True)
            _set_motor_target(config.MOTOR_TARGET_STEPS)  # 3000 유지

            if dist >= 0 and dist <= config.HARD_LOCK_DIST_MM:
                if dist_lock_start is None:
                    dist_lock_start = now
                    print(f"\n[SM] ToF {int(dist)}mm ≤ {config.HARD_LOCK_DIST_MM}mm 감지, "
                          f"{config.HARD_LOCK_HOLD_S}s 카운트 시작")
                elif now - dist_lock_start >= config.HARD_LOCK_HOLD_S:
                    dist_lock_start = None
                    _set_motor_target(0)  # 모터 복귀 시작
                    _state("HARD_LOCK")
            else:
                if dist_lock_start is not None:
                    print("\n[SM] 거리 초과 → 타이머 리셋")
                dist_lock_start = None

        # ═══════════════════════════════════════════════
        # State 4: HARD_LOCK
        #   조건: ToF ≤ 50mm (5s)
        #   행동: 전자석 ON, 모터 step 3000→0
        #   전이: 모터 step 0 도달 후 2s → DOCKED
        # ═══════════════════════════════════════════════
        elif state == "HARD_LOCK":
            _set_magnet(True)
            # 모터 target은 SOFT_CAPTURE 전이 시점에 이미 0으로 설정됨

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
        #   조건: 모터 step 0 도달 후 2s
        #   행동: 전자석 OFF, 모터 step 0 유지
        #   전이: 없음 (수동 RESET만 가능)
        # ═══════════════════════════════════════════════
        elif state == "DOCKED":
            _set_magnet(False)
            _set_motor_target(0)
            # 종료 상태 — 아무것도 하지 않음

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

        # 이전 프레임과 동일하면 스킵 (중복 전송 방지)
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
    """상태머신을 PRE_DOCKING으로 초기화 (웹 UI 버튼 또는 curl로 호출)."""
    global _dock_state, _motor_target, _magnet_on
    _set_magnet(False)
    _set_motor_target(0)
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
            "mission_time":    f"T+ {h:02d}:{m:02d}:{s:02d}",
            "marker_detected": _marker_detected,
            "marker_angle":    round(_marker_angle, 1),
            "marker_offset_x": round(_marker_offset_x, 1),
            "marker_offset_y": round(_marker_offset_y, 1),
        })

# ════════════════════════════════════════════════════════
# ── HTML 페이지 ─────────────────────────────────────────
# ════════════════════════════════════════════════════════

HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>CubeSat Docking Control</title>
<style>
* { margin:0; padding:0; box-sizing:border-box; }
body { background:#0a0a0f; color:#e0e0e0; font-family:'Courier New',monospace;
       height:100vh; display:flex; flex-direction:column; overflow:hidden; }

#topbar { background:#111118; border-bottom:1px solid #1a1a2e; padding:6px 20px;
          display:flex; justify-content:space-between; align-items:center; }
#topbar .title { font-size:11px; color:#555; letter-spacing:3px; }
#topbar .date  { font-size:11px; color:#555; }

#main { flex:1; position:relative; overflow:hidden; display:flex;
        align-items:center; justify-content:center; }
#feed { width:100%; height:100%; object-fit:contain; }

/* 공통 오버레이 */
.overlay { position:absolute; background:rgba(5,5,15,0.88); border:1px solid #1a2a4a;
           padding:14px 20px; border-radius:6px; backdrop-filter:blur(4px); }

/* 거리 (좌상단) */
#dist-box { top:20px; left:20px; min-width:190px; }
#dist-box .lbl { font-size:10px; color:#4a6a9a; letter-spacing:3px; margin-bottom:4px; }
#dist-box .val { font-size:52px; font-weight:bold; color:#4fc3f7; line-height:1; }
#dist-box .unt { font-size:12px; color:#4a6a9a; margin-top:2px; }
#dist-bar-wrap { margin-top:10px; height:4px; background:#1a2a3a; border-radius:2px; }
#dist-bar { height:100%; width:0%; background:#4fc3f7; border-radius:2px; transition:width 0.4s; }

/* 도킹 상태 (우상단) */
#status-box { top:20px; right:20px; min-width:200px; text-align:right; border-color:#2a3a1a; }
#status-box .lbl { font-size:10px; color:#4a7a3a; letter-spacing:3px; margin-bottom:6px; }
#status-box .val { font-size:20px; font-weight:bold; letter-spacing:2px; }
#dots { display:flex; gap:7px; margin-top:10px; justify-content:flex-end; }
.dot { width:11px; height:11px; border-radius:50%; background:#1a2a1a; transition:background 0.3s; }
.dot.on { background:#69f0ae; }

/* ArUco (좌하단) */
#aruco-box { bottom:90px; left:20px; min-width:200px; border-color:#2a1a3a; }
#aruco-box .lbl { font-size:10px; color:#7a4a9a; letter-spacing:3px; margin-bottom:8px; }
.ar-row { display:flex; justify-content:space-between; align-items:center; margin-bottom:5px; }
.ar-key { font-size:10px; color:#555; letter-spacing:2px; }
.ar-val { font-size:17px; font-weight:bold; color:#ce93d8; }
.ar-val.ok   { color:#69f0ae; }
.ar-val.warn { color:#ffb74d; }
#abar-wrap { margin-top:8px; height:4px; background:#1a1a2e; border-radius:2px; position:relative; }
#abar-zero { position:absolute; left:50%; top:0; height:100%; width:1px; background:#444; }
#abar      { position:absolute; top:0; height:100%; background:#ce93d8; border-radius:2px; transition:all 0.3s; }

/* 모터 진행 (우하단) */
#motor-box { bottom:90px; right:20px; min-width:190px; text-align:right; border-color:#1a2a1a; }
#motor-box .lbl { font-size:10px; color:#3a6a4a; letter-spacing:3px; margin-bottom:6px; }
#motor-val { font-size:36px; font-weight:bold; color:#69f0ae; }
#motor-sub { font-size:10px; color:#3a5a3a; margin-top:2px; }
#motor-bar-wrap { margin-top:10px; height:4px; background:#1a2a1a; border-radius:2px; }
#motor-bar { height:100%; width:0%; background:#69f0ae; border-radius:2px; transition:width 0.4s; }

/* 하단 바 */
#bottombar { background:#0d0d18; border-top:1px solid #1a1a2e; display:flex; align-items:stretch; }
.bsec { padding:12px 22px; display:flex; flex-direction:column; justify-content:center; }
.bsec + .bsec { border-left:1px solid #1a1a2e; }
.blabel { font-size:10px; color:#444; letter-spacing:3px; margin-bottom:3px; }
.bval   { font-size:26px; color:#fff; letter-spacing:2px; }

/* 시퀀스 바 */
#seq-sec { flex:1; padding:10px 22px; border-left:1px solid #1a1a2e;
           display:flex; flex-direction:column; justify-content:center; }
#seq-bar { display:flex; justify-content:space-between; align-items:center;
           position:relative; margin-top:6px; }
#seq-line { position:absolute; top:50%; left:0; right:0; height:1px; background:#1a1a2e; }
.step { display:flex; flex-direction:column; align-items:center; gap:5px; position:relative; z-index:1; }
.sdot { width:13px; height:13px; border-radius:50%; background:#1a2a1a;
        border:1px solid #2a3a2a; transition:all 0.3s; }
.sdot.active { background:#69f0ae; border-color:#69f0ae; }
.sdot.current { background:#4fc3f7; border-color:#4fc3f7; box-shadow:0 0 6px #4fc3f7; }
.slbl { font-size:9px; color:#333; letter-spacing:1px; transition:color 0.3s; }
.slbl.active  { color:#69f0ae; }
.slbl.current { color:#4fc3f7; }

/* 전자석 */
#mag-sec { padding:12px 22px; border-left:1px solid #1a1a2e;
           display:flex; flex-direction:column; justify-content:center; align-items:flex-end; }
#mag-val { font-size:17px; letter-spacing:3px; transition:color 0.3s; }

#reset-btn {
  background: transparent; border: 1px solid #c0392b; color: #e74c3c;
  font-family: 'Courier New', monospace; font-size: 13px; letter-spacing: 2px;
  padding: 6px 14px; border-radius: 4px; cursor: pointer; transition: all 0.2s;
}
#reset-btn:hover  { background: #c0392b; color: #fff; }
#reset-btn:active { background: #922b21; }
</style>
</head>
<body>

<div id="topbar">
  <span class="title">CUBESAT DOCKING CONTROL SYSTEM  v2</span>
  <span class="date" id="clock"></span>
</div>

<div id="main">
  <img id="feed" src="/video_feed">

  <!-- 거리 -->
  <div class="overlay" id="dist-box">
    <div class="lbl">TARGET DISTANCE</div>
    <div class="val" id="dist-val">---</div>
    <div class="unt">mm</div>
    <div id="dist-bar-wrap"><div id="dist-bar"></div></div>
  </div>

  <!-- 도킹 상태 -->
  <div class="overlay" id="status-box">
    <div class="lbl">DOCKING STATUS</div>
    <div class="val" id="state-val">PRE_DOCKING</div>
    <div id="dots">
      <div class="dot" id="d1"></div>
      <div class="dot" id="d2"></div>
      <div class="dot" id="d3"></div>
      <div class="dot" id="d4"></div>
    </div>
  </div>

  <!-- ArUco -->
  <div class="overlay" id="aruco-box">
    <div class="lbl">ATTITUDE / ALIGNMENT</div>
    <div class="ar-row">
      <span class="ar-key">MARKER</span>
      <span class="ar-val" id="mk-det">NO LOCK</span>
    </div>
    <div class="ar-row">
      <span class="ar-key">ROLL</span>
      <span class="ar-val" id="mk-ang">---</span>
    </div>
    <div class="ar-row">
      <span class="ar-key">dX</span>
      <span class="ar-val" id="mk-dx">---</span>
    </div>
    <div class="ar-row">
      <span class="ar-key">dY</span>
      <span class="ar-val" id="mk-dy">---</span>
    </div>
    <div id="abar-wrap"><div id="abar-zero"></div><div id="abar"></div></div>
  </div>

  <!-- 모터 -->
  <div class="overlay" id="motor-box">
    <div class="lbl">MOTOR POSITION</div>
    <div id="motor-val">0</div>
    <div id="motor-sub">/ 3000 steps</div>
    <div id="motor-bar-wrap"><div id="motor-bar"></div></div>
  </div>
</div>

<div id="bottombar">
  <div class="bsec">
    <div class="blabel">MISSION TIME</div>
    <div class="bval" id="mtime">T+ 00:00:00</div>
  </div>

  <div id="seq-sec">
    <div class="blabel">SEQUENCE</div>
    <div id="seq-bar">
      <div id="seq-line"></div>
      <div class="step"><div class="sdot" id="st0"></div><span class="slbl" id="sl0">PRE</span></div>
      <div class="step"><div class="sdot" id="st1"></div><span class="slbl" id="sl1">T.LOCK</span></div>
      <div class="step"><div class="sdot" id="st2"></div><span class="slbl" id="sl2">S.CAPTURE</span></div>
      <div class="step"><div class="sdot" id="st3"></div><span class="slbl" id="sl3">H.LOCK</span></div>
      <div class="step"><div class="sdot" id="st4"></div><span class="slbl" id="sl4">DOCKED</span></div>
    </div>
  </div>

  <div id="mag-sec">
    <div class="blabel">MAGNET</div>
    <div id="mag-val">OFF</div>
  </div>

  <div class="bsec" style="border-left:1px solid #1a1a2e;">
    <div class="blabel">CONTROL</div>
    <button id="reset-btn" onclick="doReset()">⟳ RESET</button>
  </div>
</div>

<script>
const STATES = ["PRE_DOCKING","TARGET_LOCK","SOFT_CAPTURE","HARD_LOCK","DOCKED"];
const STATE_COLOR = {
  PRE_DOCKING:  "#a0a0a0",
  TARGET_LOCK:  "#4fc3f7",
  SOFT_CAPTURE: "#69f0ae",
  HARD_LOCK:    "#ff9800",
  DOCKED:       "#69f0ae",
};

async function update() {
  try {
    const d = await fetch('/api/status').then(r => r.json());

    // 미션 시간
    document.getElementById('mtime').textContent = d.mission_time;

    // 거리
    const dist = d.distance_mm;
    document.getElementById('dist-val').textContent = dist >= 0 ? Math.round(dist) : '---';
    const dpct = dist >= 0 ? Math.min(100, Math.round((1 - dist / 500) * 100)) : 0;
    document.getElementById('dist-bar').style.width = dpct + '%';

    // 상태
    const sv = document.getElementById('state-val');
    sv.textContent = d.state;
    sv.style.color = STATE_COLOR[d.state] || '#fff';

    const idx = STATES.indexOf(d.state);
    for (let i = 0; i < 4; i++)
      document.getElementById('d'+(i+1)).classList.toggle('on', i < idx);
    for (let i = 0; i < 5; i++) {
      const dot = document.getElementById('st'+i);
      const lbl = document.getElementById('sl'+i);
      dot.className = 'sdot' + (i < idx ? ' active' : i === idx ? ' current' : '');
      lbl.className = 'slbl' + (i < idx ? ' active' : i === idx ? ' current' : '');
    }

    // 전자석
    const mv = document.getElementById('mag-val');
    mv.textContent = d.magnet ? 'ON' : 'OFF';
    mv.style.color = d.magnet ? '#69f0ae' : '#e74c3c';

    // 모터
    const steps = d.motor_steps;
    document.getElementById('motor-val').textContent = steps;
    const mpct = Math.min(100, Math.round(steps / 3000 * 100));
    document.getElementById('motor-bar').style.width = mpct + '%';
    const motorColor = steps >= 3000 ? '#69f0ae' : steps > 0 ? '#4fc3f7' : '#3a5a3a';
    document.getElementById('motor-bar').style.background = motorColor;
    document.getElementById('motor-val').style.color = motorColor;

    // ArUco
    const mkDet = document.getElementById('mk-det');
    const mkAng = document.getElementById('mk-ang');
    const mkDx  = document.getElementById('mk-dx');
    const mkDy  = document.getElementById('mk-dy');
    const abar  = document.getElementById('abar');

    if (d.marker_detected) {
      const ang = d.marker_angle;
      mkDet.textContent = 'LOCKED';   mkDet.className = 'ar-val ok';
      const angOk = Math.abs(ang) < 5;
      mkAng.textContent = (ang >= 0 ? '+' : '') + ang.toFixed(1) + '°';
      mkAng.className   = 'ar-val ' + (angOk ? 'ok' : 'warn');
      const dxOk = Math.abs(d.marker_offset_x) < 20;
      const dyOk = Math.abs(d.marker_offset_y) < 20;
      mkDx.textContent = (d.marker_offset_x >= 0 ? '+' : '') + d.marker_offset_x.toFixed(0) + ' px';
      mkDx.className   = 'ar-val ' + (dxOk ? 'ok' : 'warn');
      mkDy.textContent = (d.marker_offset_y >= 0 ? '+' : '') + d.marker_offset_y.toFixed(0) + ' px';
      mkDy.className   = 'ar-val ' + (dyOk ? 'ok' : 'warn');
      const clamp = Math.max(-45, Math.min(45, ang));
      const bp    = (clamp + 45) / 90 * 100;
      abar.style.left       = Math.min(50, bp) + '%';
      abar.style.width      = Math.abs(bp - 50) + '%';
      abar.style.background = angOk ? '#69f0ae' : '#ffb74d';
    } else {
      mkDet.textContent = 'NO LOCK'; mkDet.className = 'ar-val';
      mkAng.textContent = '---';     mkAng.className = 'ar-val';
      mkDx.textContent  = '---';     mkDx.className  = 'ar-val';
      mkDy.textContent  = '---';     mkDy.className  = 'ar-val';
      abar.style.width = '0%';
    }
  } catch(e) {}
}

async function doReset() {
  const btn = document.getElementById('reset-btn');
  btn.textContent = '...';
  btn.disabled = true;
  try {
    await fetch('/api/reset', { method: 'POST' });
    await update();
  } catch(e) {}
  setTimeout(() => { btn.textContent = '⟳ RESET'; btn.disabled = false; }, 800);
}

setInterval(update, 500);
setInterval(() => {
  document.getElementById('clock').textContent = new Date().toLocaleString('ko-KR');
}, 1000);
update();
</script>
</body>
</html>
"""

# ════════════════════════════════════════════════════════
# ── 메인 실행 ───────────────────────────────────────────
# ════════════════════════════════════════════════════════
if __name__ == '__main__':
    threading.Thread(target=tof_thread,           daemon=True, name="ToF").start()
    threading.Thread(target=camera_thread,        daemon=True, name="Camera").start()
    threading.Thread(target=motor_thread,         daemon=True, name="Motor").start()
    threading.Thread(target=state_machine_thread, daemon=True, name="StateMachine").start()

    print("=" * 52)
    print("  CubeSat Docking System  |  main_ver1.py")
    print(f"  웹 UI : http://pi.local:5000")
    print("=" * 52)

    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        _set_magnet(False)
        _set_motor_target(0)
        if GPIO_OK:
            _pwm.stop()
            GPIO.cleanup()
        print("\n[SYS] 종료 완료")
