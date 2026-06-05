# web_server.py — Flask 웹 서버, API 라우트, HTML UI

import time
import cv2
from flask import Flask, Response, jsonify
import shared
# from shared import _lock  # shared._lock 직접 사용
from constants import SERVO_STOWED_ANGLE, SERVO_CHARGING_ANGLE
from hardware import _set_magnet, _set_motor_target, _set_servo, _cancel_approval
import shared  # _approved, _pending_transition 은 shared 에서 직접 접근

app = Flask(__name__)

def gen_frames():
    """camera_thread가 갱신하는 _last_frame을 JPEG로 인코딩해 스트리밍."""
    prev_frame = None
    while True:
        with shared._lock:
            frame = shared._last_frame

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

@app.route('/api/approve', methods=['POST'])
def api_approve():
    """대기 중인 상태 전이 승인."""
    with shared._lock:
        pending = shared._pending_transition
        if pending is None:
            return jsonify({"ok": False, "msg": "승인 대기 중인 전이 없음"})
        shared._approved = True
    print(f"\n[SM] ✔  [{pending}] 전이 승인됨")
    return jsonify({"ok": True, "approved": pending})

@app.route('/api/reject', methods=['POST'])
def api_reject():
    """대기 중인 상태 전이 거부."""
    with shared._lock:
        pending = shared._pending_transition
    _cancel_approval()
    print(f"\n[SM] ✗  [{pending}] 전이 거부됨")
    return jsonify({"ok": True, "rejected": pending})

@app.route('/api/reset', methods=['POST'])
def api_reset():
    """상태머신을 PRE_DOCKING으로 초기화."""
    _cancel_approval()
    _set_magnet(False)
    _set_motor_target(0)
    _set_servo(SERVO_STOWED_ANGLE)
    with shared._lock:
        shared._dock_state = "PRE_DOCKING"
    print("\n[SM] ★ 수동 리셋 → PRE_DOCKING")
    return jsonify({"ok": True, "state": "PRE_DOCKING"})

@app.route('/api/status')
def api_status():
    with shared._lock:
        elapsed = int(time.time() - shared._start_time)
        h = elapsed // 3600
        m = (elapsed % 3600) // 60
        s = elapsed % 60
        return jsonify({
            "distance_mm":        round(shared._distance_mm, 1),
            "state":              shared._dock_state,
            "magnet":             shared._magnet_on,
            "motor_steps":        shared._motor_steps,
            "motor_target":       shared._motor_target,
            "servo_angle":        round(shared._servo_angle, 1),
            "mission_time":       f"T+ {h:02d}:{m:02d}:{s:02d}",
            "marker_detected":    shared._marker_detected,
            "marker_angle":       round(shared._marker_angle, 1),
            "marker_offset_x":    round(shared._marker_offset_x, 1),
            "marker_offset_y":    round(shared._marker_offset_y, 1),
            "pending_transition": shared._pending_transition,
        })

# ════════════════════════════════════════════════════════
# ── HTML 페이지 (orbital-command UI 스타일) ─────────────
# ════════════════════════════════════════════════════════

HTML_PAGE = """<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>CUBESAT_OS v4.0</title>
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

/* HUD 코너 패널 — 카메라 위 오버레이 */
.hud-panel {
  position: absolute;
  background: rgba(0,0,0,0.58);
  border: 1px solid rgba(255,255,255,0.1);
  padding: 6px 10px;
  pointer-events: none;
  user-select: none;
  z-index: 8;
  backdrop-filter: blur(2px);
}
.hud-panel .hp-lbl {
  font-size: 8px;
  font-weight: 700;
  letter-spacing: 3px;
  color: rgba(0,238,252,0.55);
  margin-bottom: 3px;
}
.hud-panel .hp-val {
  font-size: 18px;
  font-weight: 700;
  letter-spacing: 1px;
  color: #fff;
  transition: color 0.3s;
}
.hud-tl { top: 12px; left: 12px; }
.hud-tr { top: 12px; right: 12px; text-align: right; }
.hud-bl { bottom: 12px; left: 12px; }
.hud-bc {
  bottom: 12px;
  left: 50%;
  transform: translateX(-50%);
  text-align: center;
  white-space: nowrap;
}

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

/* ── APPROVAL BAR ────────────────────────────── */
#approval-bar {
  display: none;
  position: fixed;
  top: 48px; left: 0; right: 0;
  z-index: 100;
  background: rgba(10, 8, 0, 0.96);
  border-bottom: 2px solid var(--amber);
  padding: 10px 20px;
  align-items: center;
  justify-content: space-between;
  gap: 16px;
  backdrop-filter: blur(4px);
  animation: slideDown 0.25s ease-out;
}
@keyframes slideDown { from { transform: translateY(-100%); opacity:0; } to { transform: translateY(0); opacity:1; } }
#approval-bar.visible { display: flex; }
.apv-left { display:flex; align-items:center; gap:12px; }
.apv-icon { font-size:18px; animation: blink 0.8s infinite; }
.apv-msg { font-size:10px; font-weight:700; letter-spacing:3px; color:var(--amber); }
.apv-state { font-size:14px; font-weight:700; letter-spacing:2px; color:#fff; }
.apv-right { display:flex; gap:10px; }
.apv-btn {
  padding: 8px 20px; border: 1px solid;
  background: transparent; cursor: pointer;
  font-family: inherit; font-size: 11px; font-weight: 700;
  letter-spacing: 2px; transition: all 0.15s;
}
.apv-btn-approve { border-color: var(--green); color: var(--green); }
.apv-btn-approve:hover { background: var(--green); color: #000; }
.apv-btn-reject  { border-color: #c0392b; color: #e74c3c; }
.apv-btn-reject:hover  { background: #c0392b; color: #fff; }

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

<!-- ═══ APPROVAL BAR (반자동 승인) ══════════════════════ -->
<div id="approval-bar">
  <div class="apv-left">
    <span class="apv-icon">⏸</span>
    <div>
      <div class="apv-msg">AWAITING OPERATOR APPROVAL — NEXT STATE</div>
      <div class="apv-state" id="apv-state-name">---</div>
    </div>
  </div>
  <div class="apv-right" style="align-items:center;gap:14px;">
    <span style="font-size:9px;color:#666;letter-spacing:2px;">KEYBOARD: ENTER</span>
    <button class="apv-btn apv-btn-approve" onclick="doApprove()" id="apv-approve-btn">↵ APPROVE</button>
    <button class="apv-btn apv-btn-reject"  onclick="doReject()">✗ REJECT</button>
  </div>
</div>

<!-- ═══ TOP BAR ══════════════════════════════════════════ -->
<header id="topbar">
  <div style="display:flex;align-items:center;gap:16px;">
    <span class="sys-name">CUBESAT_OS_v4.0</span>
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

          <!-- HUD: 거리 (좌상단) -->
          <div class="hud-panel hud-tl">
            <div class="hp-lbl">DIST</div>
            <div class="hp-val" id="hud-dist">--- mm</div>
          </div>

          <!-- HUD: 접근속도 (우상단) -->
          <div class="hud-panel hud-tr">
            <div class="hp-lbl">RATE</div>
            <div class="hp-val" id="hud-rate">+0.0 mm/s</div>
          </div>

          <!-- HUD: 현재 상태 (좌하단) -->
          <div class="hud-panel hud-bl">
            <div class="hp-lbl">STATE</div>
            <div class="hp-val" id="hud-state" style="font-size:13px;letter-spacing:2px;">PRE_DOCKING</div>
          </div>

          <!-- HUD: 미션 시간 (하단 중앙) -->
          <div class="hud-panel hud-bc">
            <div class="hp-lbl">MISSION_ELAPSED_TIME</div>
            <div class="hp-val" id="hud-mtime" style="font-size:14px;text-align:center;">T+ 00:00:00</div>
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
            STOWED: 0 deg &nbsp;|&nbsp; CHARGING: 65 deg
          </div>
        </div>
        <div class="pw-card" style="grid-column:span 2;">
          <h3>STATE MACHINE OVERVIEW</h3>
          <div class="sm-grid" id="sm-grid" style="grid-template-columns:repeat(4,1fr);">
            <div class="sm-card" id="sm0">
              <div class="sm-idx">STATE 1</div>
              <div class="sm-name sc-PRE_DOCKING">PRE_DOCKING</div>
              <div class="sm-act">MAG: OFF<br>MOT: 6000<br>SVO: 0°</div>
            </div>
            <div class="sm-card" id="sm1">
              <div class="sm-idx">STATE 2</div>
              <div class="sm-name sc-SOFT_CAPTURE">SOFT_CAPTURE</div>
              <div class="sm-act">MAG: ON<br>MOT: 6000</div>
            </div>
            <div class="sm-card" id="sm2">
              <div class="sm-idx">STATE 3</div>
              <div class="sm-name sc-HARD_LOCK">HARD_LOCK</div>
              <div class="sm-act">MAG: ON<br>MOT: 6000&rarr;0</div>
            </div>
            <div class="sm-card" id="sm3">
              <div class="sm-idx">STATE 4</div>
              <div class="sm-name sc-DOCKED">DOCKED</div>
              <div class="sm-act">MAG: OFF<br>MOT: 0</div>
            </div>
            <div class="sm-card" id="sm4">
              <div class="sm-idx">STATE 5</div>
              <div class="sm-name sc-CHARGING">CHARGING</div>
              <div class="sm-act">MAG: OFF<br>SVO: 65°</div>
            </div>
            <div class="sm-card" id="sm5">
              <div class="sm-idx">STATE 6</div>
              <div class="sm-name" style="color:#ff6b6b;">SEPARATION_1</div>
              <div class="sm-act">MAG: ON<br>SVO: 0°</div>
            </div>
            <div class="sm-card" id="sm6">
              <div class="sm-idx">STATE 7</div>
              <div class="sm-name" style="color:#ff4500;">SEPARATION_2</div>
              <div class="sm-act">MAG: ON<br>MOT: 0&rarr;6000</div>
            </div>
            <div class="sm-card" id="sm7">
              <div class="sm-idx">STATE 8</div>
              <div class="sm-name" style="color:#c0392b;">SEPARATION_3</div>
              <div class="sm-act">MAG: OFF<br>MOT: 6000</div>
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
      <div class="seq-step"><div class="seq-dot" id="sd1"></div><span class="seq-lbl" id="sl1">S.CAP</span></div>
      <div class="seq-step"><div class="seq-dot" id="sd2"></div><span class="seq-lbl" id="sl2">H.LOCK</span></div>
      <div class="seq-step"><div class="seq-dot" id="sd3"></div><span class="seq-lbl" id="sl3">DOCKED</span></div>
      <div class="seq-step"><div class="seq-dot" id="sd4"></div><span class="seq-lbl" id="sl4">CHG</span></div>
      <div class="seq-step"><div class="seq-dot" id="sd5"></div><span class="seq-lbl" id="sl5">SEP1</span></div>
      <div class="seq-step"><div class="seq-dot" id="sd6"></div><span class="seq-lbl" id="sl6">SEP2</span></div>
      <div class="seq-step"><div class="seq-dot" id="sd7"></div><span class="seq-lbl" id="sl7">SEP3</span></div>
    </div>
  </div>

  <button id="reset-btn" onclick="doReset()">&#x27F3; RESET</button>
</footer>

<script>
// ─────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────
const STATES = ['PRE_DOCKING','SOFT_CAPTURE','HARD_LOCK','DOCKED','CHARGING',
                'SEPARATION_1','SEPARATION_2','SEPARATION_3'];

const STATE_COLORS = {
  PRE_DOCKING:   '#a0a0a0',
  SOFT_CAPTURE:  '#00eefc',
  HARD_LOCK:     '#ff9800',
  DOCKED:        '#00FF55',
  CHARGING:      '#ffc832',
  SEPARATION_1:  '#ff6b6b',
  SEPARATION_2:  '#ff4500',
  SEPARATION_3:  '#c0392b',
};

const STATUS_CFG = {
  PRE_DOCKING:  { dot:'#555',   border:'rgba(100,100,100,0.3)', bg:'rgba(100,100,100,0.08)', label:'MISSION_STATUS: STANDBY' },
  SOFT_CAPTURE: { dot:'#00eefc',border:'rgba(0,238,252,0.3)',   bg:'rgba(0,238,252,0.08)',   label:'STATUS: SOFT_CAPTURE ENGAGED' },
  HARD_LOCK:    { dot:'#ff9800',border:'rgba(255,152,0,0.3)',   bg:'rgba(255,152,0,0.08)',   label:'STATUS: HARD_LOCK — RETRACTING' },
  DOCKED:       { dot:'#00FF55',border:'rgba(0,255,85,0.3)',    bg:'rgba(0,255,85,0.08)',    label:'MISSION_STATUS: DOCKED' },
  CHARGING:     { dot:'#ffc832',border:'rgba(255,200,50,0.3)',  bg:'rgba(255,200,50,0.08)',  label:'MISSION_STATUS: CHARGING ACTIVE' },
  SEPARATION_1: { dot:'#ff6b6b',border:'rgba(255,107,107,0.3)',bg:'rgba(255,107,107,0.08)', label:'STATUS: SEPARATION — SERVO RELEASE' },
  SEPARATION_2: { dot:'#ff4500',border:'rgba(255,69,0,0.3)',   bg:'rgba(255,69,0,0.08)',    label:'STATUS: SEPARATION — MOTOR EXTEND' },
  SEPARATION_3: { dot:'#c0392b',border:'rgba(192,57,43,0.3)',  bg:'rgba(192,57,43,0.08)',   label:'MISSION_STATUS: SEPARATION COMPLETE' },
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

    // ── 승인 바 ──────────────────────────────────
    const pending = d.pending_transition;
    const apvBar  = document.getElementById('approval-bar');
    if (pending) {
      document.getElementById('apv-state-name').textContent = '→  ' + pending;
      apvBar.classList.add('visible');
    } else {
      apvBar.classList.remove('visible');
    }

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

    // ── HUD 코너 패널 업데이트 ────────────────
    const isHighSpeed = (dist >= 0 && dist < 150 && velocity < -20);
    const rateStr = (velocity >= 0 ? '+' : '') + velocity.toFixed(1) + ' mm/s';

    // DIST
    const hudDist = document.getElementById('hud-dist');
    hudDist.textContent = dist >= 0 ? Math.round(dist) + ' mm' : '--- mm';
    hudDist.style.color = isDocked ? '#00eefc' : dist >= 0 && dist <= 300 ? '#ffb74d' : '#fff';

    // RATE
    const hudRate = document.getElementById('hud-rate');
    hudRate.textContent = rateStr;
    hudRate.style.color = isHighSpeed ? '#f44336' : Math.abs(velocity) > 5 ? '#f06292' : '#fff';

    // STATE
    const hudState = document.getElementById('hud-state');
    hudState.textContent = state;
    hudState.style.color = stateColor;

    // MISSION TIME
    document.getElementById('hud-mtime').textContent = mtime;

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
    for (let i = 0; i < 8; i++) {
      const dot = document.getElementById('sd' + i);
      const lbl = document.getElementById('sl' + i);
      if (!dot) continue;
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
    for (let i = 0; i < 8; i++) {
      const card = document.getElementById('sm' + i);
      if (!card) continue;
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
// Approve / Reject
// ─────────────────────────────────────────────
async function doApprove() {
  const btn = document.querySelector('.apv-btn-approve');
  btn.textContent = '...'; btn.disabled = true;
  try { await fetch('/api/approve', { method: 'POST' }); await update(); }
  catch(e) {}
  setTimeout(() => { btn.textContent = '✔ APPROVE'; btn.disabled = false; }, 500);
}

async function doReject() {
  const btn = document.querySelector('.apv-btn-reject');
  btn.textContent = '...'; btn.disabled = true;
  try { await fetch('/api/reject', { method: 'POST' }); await update(); }
  catch(e) {}
  setTimeout(() => { btn.textContent = '✗ REJECT'; btn.disabled = false; }, 500);
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

// ── 키보드 단축키: Enter → APPROVE ──────────────
document.addEventListener('keydown', (e) => {
  if (e.key === 'Enter' && !e.repeat) {
    const bar = document.getElementById('approval-bar');
    if (bar && bar.classList.contains('visible')) {
      e.preventDefault();
      doApprove();
    }
  }
});
</script>
</body>
</html>"""

# ════════════════════════════════════════════════════════
# ── 메인 실행 ─────────────────────────────