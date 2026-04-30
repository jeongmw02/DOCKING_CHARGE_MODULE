"""
web_ui_com.py
=========
CubeSat 도킹 시스템 웹 UI + ArUco 마커 인식 통합본
실행: python3 web_ui_com.py
접속: http://pi.local:5000
"""

import time
import threading
import io
from flask import Flask, Response, jsonify

try:
    from picamera2 import Picamera2
    import cv2
    import cv2.aruco as aruco
    CAMERA_OK = True
except ImportError:
    CAMERA_OK = False

try:
    import board, busio, adafruit_vl53l0x
    TOF_OK = True
except ImportError:
    TOF_OK = False

app = Flask(__name__)

# ── 전역 상태 ───────────────────────────────────────────
_lock        = threading.Lock()
_distance_mm = -1.0
_dock_state  = "IDLE"   # IDLE / APPROACH / CAPTURE / LOCK / DOCKED
_magnet      = False
_start_time  = time.time()

# ── ToF 센서 스레드 ─────────────────────────────────────
def tof_thread():
    global _distance_mm
    if not TOF_OK:
        while True:
            time.sleep(0.5)
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = adafruit_vl53l0x.VL53L0X(i2c)
        while True:
            try:
                with _lock:
                    _distance_mm = float(sensor.range)
            except Exception:
                with _lock:
                    _distance_mm = -1.0
            time.sleep(0.1)
    except Exception as e:
        print(f"[ToF] 초기화 실패: {e}")

# ── 카메라 프레임 생성기 (ArUco 인식 추가) ────────────────────────
def gen_frames():
    if not CAMERA_OK:
        while True:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + b'' + b'\r\n')
            time.sleep(0.1)
        return

    cam = Picamera2()
    cfg = cam.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    cam.configure(cfg)
    cam.start()
    time.sleep(1)

    # ArUco 마커 설정 (OpenCV 버전 호환)
    try:
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary, parameters)
    except AttributeError:
        dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        detector = None # 구버전 호환용 플래그

    while True:
        frame = cam.capture_array()

        # 1. ArUco 마커 찾기
        if detector:
            corners, ids, rejected = detector.detectMarkers(frame)
        else:
            corners, ids, rejected = aruco.detectMarkers(frame, dictionary, parameters=parameters)

        # 2. 화면에 마커 박스 그리기
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            
            # 마커 중심점 계산 및 표시 (도킹 타겟팅용)
            for i in range(len(ids)):
                c = corners[i][0]
                center_x = int((c[0][0] + c[2][0]) / 2)
                center_y = int((c[0][1] + c[2][1]) / 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                cv2.putText(frame, "TARGET LOCKED", (center_x - 50, center_y - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 3. 거리 및 상태 오버레이 (기존 코드)
        with _lock:
            dist  = _distance_mm
            state = _dock_state

        if dist >= 0:
            dist_text = f"{int(dist)} mm"
            color = (100, 220, 100) if dist > 100 else (50, 100, 255)
        else:
            dist_text = "N/A"
            color = (120, 120, 120)

        cv2.putText(frame, dist_text, (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 2)
        cv2.putText(frame, state, (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 1)

        # 십자선 (크로스헤어) 그리기 - 화면 중앙 (우주선 도킹 UI 느낌)
        h, w, _ = frame.shape
        cv2.line(frame, (w//2 - 20, h//2), (w//2 + 20, h//2), (255, 255, 255), 1)
        cv2.line(frame, (w//2, h//2 - 20), (w//2, h//2 + 20), (255, 255, 255), 1)

        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
               + buf.tobytes() + b'\r\n')

# ── Flask 라우트 ────────────────────────────────────────
@app.route('/')
def index():
    return HTML_PAGE

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def api_status():
    with _lock:
        elapsed = int(time.time() - _start_time)
        h = elapsed // 3600
        m = (elapsed % 3600) // 60
        s = elapsed % 60
        return jsonify({
            "distance_mm": round(_distance_mm, 1),
            "state":       _dock_state,
            "magnet":      _magnet,
            "mission_time": f"T+ {h:02d}:{m:02d}:{s:02d}"
        })

# ── HTML 페이지 ─────────────────────────────────────────
HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Docking Control</title>
<style>
* { margin:0; padding:0; box-sizing:border-box; }
body { background:#0a0a0f; color:#e0e0e0; font-family:'Courier New',monospace; height:100vh; display:flex; flex-direction:column; overflow:hidden; }

#topbar { background:#111118; border-bottom:1px solid #1a1a2e; padding:6px 20px; display:flex; justify-content:space-between; align-items:center; }
#topbar .title { font-size:11px; color:#555; letter-spacing:3px; }
#topbar .date  { font-size:11px; color:#555; }

#main { flex:1; position:relative; overflow:hidden; display:flex; align-items:center; justify-content:center; }
#feed { width:100%; height:100%; object-fit:contain; }

.overlay { position:absolute; background:rgba(5,5,15,0.88); border:1px solid #1a2a4a; padding:16px 22px; border-radius:6px; backdrop-filter:blur(4px); }

#dist-box { top:20px; left:20px; min-width:200px; }
#dist-box .label { font-size:11px; color:#4a6a9a; letter-spacing:3px; margin-bottom:6px; }
#dist-box .value { font-size:56px; font-weight:bold; color:#4fc3f7; line-height:1; }
#dist-box .unit  { font-size:13px; color:#4a6a9a; margin-top:2px; }
#dist-bar-wrap { margin-top:12px; height:4px; background:#1a2a3a; border-radius:2px; }
#dist-bar { height:100%; width:0%; background:#4fc3f7; border-radius:2px; transition:width 0.4s; }

#status-box { top:20px; right:20px; min-width:200px; text-align:right; border-color:#2a3a1a; }
#status-box .label  { font-size:11px; color:#4a7a3a; letter-spacing:3px; margin-bottom:8px; }
#status-box .value  { font-size:22px; font-weight:bold; letter-spacing:3px; }
#dots { display:flex; gap:8px; margin-top:12px; justify-content:flex-end; }
.dot { width:12px; height:12px; border-radius:50%; background:#1a2a1a; transition:background 0.3s; }
.dot.on { background:#69f0ae; }

#bottombar { background:#0d0d18; border-top:1px solid #1a1a2e; display:flex; align-items:stretch; }

.bsec { padding:14px 24px; display:flex; flex-direction:column; justify-content:center; }
.bsec + .bsec { border-left:1px solid #1a1a2e; }
.blabel { font-size:10px; color:#444; letter-spacing:3px; margin-bottom:4px; }
.bval   { font-size:28px; color:#fff; letter-spacing:2px; }

#seq-sec { flex:1; padding:10px 24px; border-left:1px solid #1a1a2e; display:flex; flex-direction:column; justify-content:center; }
#seq-bar { display:flex; justify-content:space-between; align-items:center; position:relative; margin-top:6px; }
#seq-line { position:absolute; top:50%; left:0; right:0; height:1px; background:#1a1a2e; }
.step { display:flex; flex-direction:column; align-items:center; gap:6px; position:relative; z-index:1; }
.step-dot { width:14px; height:14px; border-radius:50%; background:#1a2a1a; border:1px solid #2a3a2a; transition:all 0.3s; }
.step-dot.active { background:#69f0ae; border-color:#69f0ae; }
.step-lbl { font-size:10px; color:#333; letter-spacing:1px; transition:color 0.3s; }
.step-lbl.active { color:#69f0ae; }

#mag-sec { padding:14px 24px; border-left:1px solid #1a1a2e; display:flex; flex-direction:column; justify-content:center; align-items:flex-end; }
#mag-val { font-size:18px; letter-spacing:3px; transition:color 0.3s; }
</style>
</head>
<body>

<div id="topbar">
  <span class="title">CUBESAT DOCKING CONTROL SYSTEM</span>
  <span class="date" id="clock"></span>
</div>

<div id="main">
  <img id="feed" src="/video_feed">

  <div class="overlay" id="dist-box">
    <div class="label">TARGET DISTANCE</div>
    <div class="value" id="dist-val">---</div>
    <div class="unit">mm</div>
    <div id="dist-bar-wrap"><div id="dist-bar"></div></div>
  </div>

  <div class="overlay" id="status-box">
    <div class="label">DOCKING STATUS</div>
    <div class="value" id="state-val">IDLE</div>
    <div id="dots">
      <div class="dot" id="d1"></div>
      <div class="dot" id="d2"></div>
      <div class="dot" id="d3"></div>
      <div class="dot" id="d4"></div>
    </div>
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
      <div class="step"><div class="step-dot" id="st0"></div><span class="step-lbl" id="sl0">IDLE</span></div>
      <div class="step"><div class="step-dot" id="st1"></div><span class="step-lbl" id="sl1">APPROACH</span></div>
      <div class="step"><div class="step-dot" id="st2"></div><span class="step-lbl" id="sl2">CAPTURE</span></div>
      <div class="step"><div class="step-dot" id="st3"></div><span class="step-lbl" id="sl3">LOCK</span></div>
      <div class="step"><div class="step-dot" id="st4"></div><span class="step-lbl" id="sl4">DOCKED</span></div>
    </div>
  </div>

  <div id="mag-sec">
    <div class="blabel">MAGNET</div>
    <div id="mag-val">OFF</div>
  </div>
</div>

<script>
const STATE_IDX = {IDLE:0, APPROACH:1, CAPTURE:2, LOCK:3, DOCKED:4};
const STATE_COLOR = {IDLE:'#69f0ae', APPROACH:'#4fc3f7', CAPTURE:'#ffeb3b', LOCK:'#ff9800', DOCKED:'#69f0ae'};

async function update() {
  try {
    const r = await fetch('/api/status');
    const d = await r.json();

    document.getElementById('mtime').textContent = d.mission_time;

    const dist = d.distance_mm;
    document.getElementById('dist-val').textContent = dist >= 0 ? Math.round(dist) : '---';
    const pct = dist >= 0 ? Math.min(100, Math.round((1 - dist/500)*100)) : 0;
    document.getElementById('dist-bar').style.width = pct + '%';

    const sv = document.getElementById('state-val');
    sv.textContent = d.state;
    sv.style.color = STATE_COLOR[d.state] || '#69f0ae';

    const idx = STATE_IDX[d.state] ?? 0;
    for (let i = 0; i < 4; i++) {
      document.getElementById('d'+(i+1)).classList.toggle('on', i < idx);
    }
    for (let i = 0; i < 5; i++) {
      const active = i <= idx;
      document.getElementById('st'+i).classList.toggle('active', active);
      document.getElementById('sl'+i).classList.toggle('active', active);
    }

    const mv = document.getElementById('mag-val');
    mv.textContent = d.magnet ? 'ON' : 'OFF';
    mv.style.color  = d.magnet ? '#69f0ae' : '#e74c3c';

  } catch(e) {}
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

# ── 실행 ────────────────────────────────────────────────
if __name__ == '__main__':
    threading.Thread(target=tof_thread, daemon=True).start()
    print("서버 시작: http://pi.local:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)
