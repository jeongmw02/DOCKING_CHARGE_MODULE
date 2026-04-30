"""
auto_dock.py
============
자동 도킹 제어 스크립트
- 카메라 웹 스트리밍 (http://pi.local:5000/video)
- ToF 거리 40mm 이하 → 전자석 자동 ON
- 터미널에서 'off' 입력 → 전자석 OFF
실행: python3 auto_dock.py
"""

import time
import threading
import RPi.GPIO as GPIO
import config
from flask import Flask, Response
from picamera2 import Picamera2
import cv2

try:
    import board, busio, adafruit_vl53l0x
    TOF_OK = True
except ImportError:
    TOF_OK = False
    print("[WARNING] ToF 라이브러리 없음 → 시뮬레이션")

# ── 전역 상태 ───────────────────────────────────────────
_distance_mm   = -1.0
_magnet_on     = False
_lock          = threading.Lock()

# ── GPIO 초기화 ─────────────────────────────────────────
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(config.ELECTROMAGNET_PIN, GPIO.OUT, initial=GPIO.LOW)
_pwm = GPIO.PWM(config.ELECTROMAGNET_PIN, 1000)
_pwm.start(0)

def magnet_on():
    global _magnet_on
    _pwm.ChangeDutyCycle(config.ELECTROMAGNET_PULL_DUTY)
    time.sleep(config.ELECTROMAGNET_PULL_MS / 1000.0)
    _pwm.ChangeDutyCycle(config.ELECTROMAGNET_HOLD_DUTY)
    _magnet_on = True
    print("[MAG] 전자석 ON (자동 흡착)")

def magnet_off():
    global _magnet_on
    _pwm.ChangeDutyCycle(0)
    _magnet_on = False
    print("[MAG] 전자석 OFF")

# ── ToF 센서 스레드 ─────────────────────────────────────
def tof_thread():
    global _distance_mm, _magnet_on
    if not TOF_OK:
        return
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_vl53l0x.VL53L0X(i2c)
        print("[ToF] 센서 초기화 완료")
        while True:
            try:
                dist = float(sensor.range)
                with _lock:
                    _distance_mm = dist
                print(f"[ToF] 거리: {int(dist)} mm", end='\r')

                # 40mm 이하 → 전자석 자동 ON
                if dist <= 40 and not _magnet_on:
                    magnet_on()

            except Exception as e:
                with _lock:
                    _distance_mm = -1.0
            time.sleep(0.1)
    except Exception as e:
        print(f"[ToF] 초기화 실패: {e}")

# ── 카메라 스트리밍 ─────────────────────────────────────
app = Flask(__name__)

def gen_frames():
    cam = Picamera2()
    cfg = cam.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    cam.configure(cfg)
    cam.start()
    time.sleep(1)
    print("[CAM] 카메라 시작")

    while True:
        frame = cam.capture_array()

        with _lock:
            dist  = _distance_mm
            mag   = _magnet_on

        # 거리 표시
        if dist >= 0:
            color = (50, 200, 50) if dist > 40 else (50, 50, 255)
            cv2.putText(frame, f"{int(dist)} mm", (20, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 2)
        else:
            cv2.putText(frame, "N/A", (20, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (120, 120, 120), 2)

        # 전자석 상태
        mag_color = (50, 255, 100) if mag else (50, 50, 200)
        mag_text  = "MAGNET: ON" if mag else "MAGNET: OFF"
        cv2.putText(frame, mag_text, (20, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, mag_color, 2)

        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
               + buf.tobytes() + b'\r\n')

@app.route('/video')
def video():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def flask_thread():
    app.run(host='0.0.0.0', port=5000, threaded=True)

# ── 메인 ────────────────────────────────────────────────
if __name__ == '__main__':
    threading.Thread(target=tof_thread, daemon=True).start()
    threading.Thread(target=flask_thread, daemon=True).start()

    print("=" * 45)
    print("자동 도킹 시스템 시작")
    print(f"카메라: http://pi.local:5000/video")
    print("명령어: 'off' → 전자석 OFF  |  'q' → 종료")
    print("=" * 45)

    while True:
        try:
            cmd = input().strip().lower()
            if cmd == 'off':
                magnet_off()
            elif cmd in ('q', 'quit'):
                magnet_off()
                _pwm.stop()
                GPIO.cleanup()
                print("종료")
                break
        except KeyboardInterrupt:
            magnet_off()
            _pwm.stop()
            GPIO.cleanup()
            print("\n종료")
            break
