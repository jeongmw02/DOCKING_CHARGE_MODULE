# sensor_core.py
import time
import threading
import math
import cv2

try:
    from picamera2 import Picamera2
    import cv2.aruco as aruco
    CAMERA_OK = True
except ImportError:
    CAMERA_OK = False

try:
    import board, busio, adafruit_vl53l0x
    TOF_OK = True
except ImportError:
    TOF_OK = False

class SensorManager:
    def __init__(self):
        self.lock = threading.Lock()
        
        # 통합 상태 데이터
        self.state_data = {
            "distance_mm": -1.0,
            "dock_state": "IDLE",
            "magnet": False,
            "marker_detected": False,
            "marker_angle": 0.0,
            "marker_offset_x": 0.0,
            "marker_offset_y": 0.0
        }
        
        self.latest_frame = None  # UI로 보낼 JPEG 프레임
        self.start_time = time.time()
        
        # 스레드 시작
        threading.Thread(target=self._tof_thread, daemon=True).start()
        threading.Thread(target=self._camera_thread, daemon=True).start()

    def get_status(self):
        """웹 서버가 상태를 물어볼 때 던져주는 함수"""
        with self.lock:
            elapsed = int(time.time() - self.start_time)
            h, m, s = elapsed // 3600, (elapsed % 3600) // 60, elapsed % 60
            
            # 딕셔너리 복사해서 반환
            status = self.state_data.copy()
            status["mission_time"] = f"T+ {h:02d}:{m:02d}:{s:02d}"
            return status

    def get_jpeg_frame(self):
        """웹 서버가 영상을 요구할 때 최신 프레임을 던져줌"""
        with self.lock:
            return self.latest_frame

    def _tof_thread(self):
        if not TOF_OK: return
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_vl53l0x.VL53L0X(i2c)
        while True:
            try:
                dist = float(sensor.range)
            except Exception:
                dist = -1.0
            
            with self.lock:
                self.state_data["distance_mm"] = dist
            time.sleep(0.1)

    def _calc_marker_angle(self, corners_single):
        c = corners_single[0]
        dx, dy = c[1][0] - c[0][0], c[1][1] - c[0][1]
        return math.degrees(math.atan2(dy, dx))

    def _camera_thread(self):
        if not CAMERA_OK: return
        
        cam = Picamera2()
        cam.configure(cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
        cam.start()
        time.sleep(1)

        # ArUco 초기화 로직
        try:
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters()
            detector = aruco.ArucoDetector(dictionary, parameters)
            use_new_api = True
        except AttributeError:
            dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters_create()
            detector = None
            use_new_api = False

        while True:
            frame = cam.capture_array()
            h, w, _ = frame.shape
            cx_img, cy_img = w // 2, h // 2

            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            if use_new_api:
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=parameters)

            detected = ids is not None and len(ids) > 0

            with self.lock:
                self.state_data["marker_detected"] = detected
                if detected:
                    aruco.drawDetectedMarkers(frame, corners, ids)
                    self.state_data["marker_angle"] = round(self._calc_marker_angle(corners[0]), 1)
                    
                    mx, my = int((corners[0][0][0][0] + corners[0][0][2][0]) / 2), int((corners[0][0][0][1] + corners[0][0][2][1]) / 2)
                    self.state_data["marker_offset_x"] = round(float(mx - cx_img), 1)
                    self.state_data["marker_offset_y"] = round(float(my - cy_img), 1)

                    # 오버레이 그리기 (최적화를 위해 꼭 필요한 것만 남김)
                    cv2.circle(frame, (mx, my), 6, (0, 255, 80), -1)
                    cv2.line(frame, (mx, my), (cx_img, cy_img), (255, 200, 0), 1)
                else:
                    self.state_data["marker_angle"] = 0.0
                    self.state_data["marker_offset_x"] = 0.0
                    self.state_data["marker_offset_y"] = 0.0

                # 프레임 압축 (JPEG) 후 저장
                _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                self.latest_frame = buf.tobytes()
