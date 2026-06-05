# shared.py — 모든 스레드가 공유하는 전역 상태

import threading
import time

_lock    = threading.Lock()
_running = True   # False로 설정 시 모든 스레드 GPIO 접근 중단

# 센서
_distance_mm     = -1.0
_marker_detected = False
_marker_angle    = 0.0
_marker_offset_x = 0.0
_marker_offset_y = 0.0

# 액추에이터
_magnet_on    = False
_motor_steps  = 6000   # 시작 시 완전 연장 위치로 가정
_motor_target = 6000   # PRE_DOCKING 유지 목표
_servo_angle  = 0      # MG992 충전 서보 현재 각도 (deg)

# 상태머신
_dock_state = "PRE_DOCKING"
_start_time = time.time()

# ── 반자동 승인 ──────────────────────────────────────────
_pending_transition = None   # 승인 대기 중인 다음 상태 이름 (str | None)
_approved           = False  # True → state_machine_thread가 전이 진행
_semi_auto          = True   # False로 바꾸면 완전 자동 복귀

# 카메라 프레임 (camera_thread → gen_frames)
_last_frame = None

