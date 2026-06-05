# threads.py — ToF 센서, 카메라, 모터 스레드

import time, math
import config
import cv2
import cv2.aruco as aruco
from constants import MOTOR_EXTENDED_STEPS, MOTOR_MIN_DELAY, MOTOR_MAX_DELAY, MOTOR_ACCEL_STEPS
import shared
from hardware import _set_servo, GPIO_OK, _servo_pwm

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

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
# ── Thread 1: ToF 센서 ──────────────────────────────────
# ════════════════════════════════════════════════════════

def tof_thread():
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
                with shared._lock:
                    shared._distance_mm = dist
            except Exception:
                with shared._lock:
                    shared._distance_mm = -1.0
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

    if not CAMERA_OK:
        print("[CAM] 카메라 라이브러리 없음, 스레드 종료")
        return

    cap = _open_usb_camera()
    if cap is None:
        print("[CAM] USB 카메라를 찾을 수 없음, 스레드 종료")
        return
    print("[CAM] USB 카메라 시작")

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

        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue
        h, w, _ = frame.shape
        cx_img, cy_img = w // 2, h // 2
        frame_count += 1

        # ArUco 감지: N프레임마다만 실행
        if frame_count % ARUCO_EVERY_N == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
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

            with shared._lock:
                shared._marker_detected = True
                shared._marker_angle    = angle
                shared._marker_offset_x = off_x
                shared._marker_offset_y = off_y

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
            with shared._lock:
                shared._marker_detected = False

        # 상태 읽기
        with shared._lock:
            dist  = shared._distance_mm
            state = shared._dock_state
            mag   = shared._magnet_on
            steps = shared._motor_steps
            tgt   = shared._motor_target

        # ── 카메라 오버레이 (최소화) ─────────────────
        state_colors = {
            "PRE_DOCKING":  (160, 160, 160),
            "TARGET_LOCK":  (100, 220, 255),
            "SOFT_CAPTURE": (50,  255, 150),
            "HARD_LOCK":    (50,  150, 255),
            "DOCKED":       (80,  255, 80),
            "CHARGING":     (255, 200, 50),
        }
        sc = state_colors.get(state, (200, 200, 200))
        # 상태 이름만 우하단에 작게 표시 (HTML HUD와 중복 방지)
        cv2.putText(frame, state, (w - 220, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, sc, 1)

        # 크로스헤어
        ch_c = (50, 255, 100) if detected else (130, 130, 130)
        cv2.line(frame, (cx_img - 28, cy_img), (cx_img + 28, cy_img), ch_c, 1)
        cv2.line(frame, (cx_img, cy_img - 28), (cx_img, cy_img + 28), ch_c, 1)
        cv2.circle(frame, (cx_img, cy_img), 32, ch_c, 1)

        with shared._lock:
            shared._last_frame = frame.copy()

# ════════════════════════════════════════════════════════
# ── Thread 3: 스테퍼 모터 ───────────────────────────────
# ════════════════════════════════════════════════════════

def motor_thread():
    """trapezoid 가감속 프로파일로 _motor_target 추종."""
    move_count  = 0
    last_target = None

    while shared._running:
        with shared._lock:
            current = shared._motor_steps
            target  = shared._motor_target

        if current == target:
            # 목표 도달 시 드라이버 비활성화
            if last_target is not None:
                if GPIO_OK and GPIO and config.STEPPER_EN_PIN is not None:
                    try:
                        GPIO.output(config.STEPPER_EN_PIN, GPIO.HIGH)
                    except Exception:
                        pass
            move_count  = 0
            last_target = None
            time.sleep(0.005)
            continue

        # 새 목표 감지 시 드라이버 활성화 + 카운터 리셋
        if target != last_target:
            move_count  = 0
            last_target = target
            if GPIO_OK and GPIO and config.STEPPER_EN_PIN is not None:
                try:
                    GPIO.output(config.STEPPER_EN_PIN, GPIO.LOW)
                except Exception:
                    pass

        direction = 1 if target > current else -1
        remaining = abs(target - current)

        if move_count < MOTOR_ACCEL_STEPS:
            delay = MOTOR_MAX_DELAY - (MOTOR_MAX_DELAY - MOTOR_MIN_DELAY) * (move_count / MOTOR_ACCEL_STEPS)
        elif remaining < MOTOR_ACCEL_STEPS:
            delay = MOTOR_MAX_DELAY - (MOTOR_MAX_DELAY - MOTOR_MIN_DELAY) * (remaining / MOTOR_ACCEL_STEPS)
        else:
            delay = MOTOR_MIN_DELAY

        half = delay / 2

        if GPIO_OK and GPIO and shared._running:
            try:
                GPIO.output(config.STEPPER_DIR_PIN,
                            GPIO.LOW if direction > 0 else GPIO.HIGH)
                GPIO.output(config.STEPPER_STEP_PIN, GPIO.HIGH)
                time.sleep(half)
                GPIO.output(config.STEPPER_STEP_PIN, GPIO.LOW)
                time.sleep(half)
            except Exception:
                break
        else:
            time.sleep(delay)

        with shared._lock:
            shared._motor_steps += direction
        move_count += 1

        if shared._motor_steps % 1000 == 0:
            print("[MOT] %d / %d steps" % (shared._motor_steps, target), end="\r")
