# app.py
from flask import Flask, Response, jsonify, render_template
import time
from sensor_core import SensorManager # 분리한 모듈 불러오기

app = Flask(__name__)
sensor_manager = SensorManager() # 백그라운드 센서 매니저 실행

def gen_frames():
    """sensor_manager에서 최신 프레임을 가져와 송출"""
    while True:
        frame = sensor_manager.get_jpeg_frame()
        if frame:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            time.sleep(0.1)

@app.route('/')
def index():
    # templates 폴더 안의 index.html을 불러옴
    return render_template('index.html') 

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def api_status():
    # 센서 코어에서 깔끔하게 정리된 데이터를 받아서 던짐
    return jsonify(sensor_manager.get_status())

if __name__ == '__main__':
    print("서버 시작: http://pi.local:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)
