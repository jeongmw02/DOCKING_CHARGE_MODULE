# state_machine.py — 8-State Semi-Auto 상태 머신

import time
import shared
from shared import _lock
from constants import *
from hardware import (_set_magnet, _set_motor_target, _set_servo,
                      _request_approval, _check_approved, _cancel_approval)

def state_machine_thread():
    """
    50ms 루프로 8-state 전이를 관리 (Semi-Auto).
    각 forward 전이 직전 _request_approval() 호출 → 웹 UI 또는 Enter로 승인.
    """

    # ── 타이머 변수 ──────────────────────────────────────
    pre_cond_start    = None  # PRE_DOCKING: ToF&ArUco 동시 조건 타이머
    dist_lock_start   = None  # SOFT_CAPTURE → HARD_LOCK 조건 타이머
    sc_enter_t        = None  # SOFT_CAPTURE 진입 시각 (타임아웃)
    hl_enter_t        = None  # HARD_LOCK 진입 시각 (타임아웃)
    motor_zero_done_t = None  # HARD_LOCK: 모터 step 0 후 타이머
    docked_enter_t    = None  # DOCKED 진입 시각
    charging_enter_t  = None  # CHARGING 진입 시각
    sep1_enter_t      = None  # SEPARATION_1 진입 시각
    sep2_enter_t      = None  # SEPARATION_2 진입 시각
    sep2_motor_done_t = None  # SEPARATION_2: 모터 6000 후 타이머

    def _state(s):
        with shared._lock:
            shared._dock_state = s
        print(f"\n[SM] ──→ {s}")

    print("[SM] 상태 머신 시작: PRE_DOCKING")

    while True:
        time.sleep(0.05)

        with shared._lock:
            state  = shared._dock_state
            dist   = shared._distance_mm
            marker = shared._marker_detected
            steps  = shared._motor_steps
            tgt    = shared._motor_target

        now = time.time()

        # ═══════════════════════════════════════════════
        # State 1: PRE_DOCKING
        # ═══════════════════════════════════════════════
        if state == "PRE_DOCKING":
            _set_magnet(False)
            _set_motor_target(MOTOR_EXTENDED_STEPS)  # 모터 6000 유지
            _set_servo(SERVO_STOWED_ANGLE)
            # 다운스트림 타이머 초기화
            dist_lock_start   = None
            sc_enter_t        = None
            hl_enter_t        = None
            motor_zero_done_t = None
            docked_enter_t    = None
            charging_enter_t  = None
            sep1_enter_t      = None
            sep2_enter_t      = None
            sep2_motor_done_t = None

            # 전이 조건: ToF ≤ 300mm AND ArUco 동시 2초
            both_ok = (dist >= 0 and dist <= SOFT_CAPTURE_DIST_MM and marker)
            if both_ok:
                if pre_cond_start is None:
                    pre_cond_start = now
                    print(f"\n[SM] 조건 충족 (ToF≤{SOFT_CAPTURE_DIST_MM}mm & ArUco), "
                          f"{SOFT_CAPTURE_HOLD_S}s 카운트...")
                elif now - pre_cond_start >= SOFT_CAPTURE_HOLD_S:
                    _request_approval("SOFT_CAPTURE")
                    if _check_approved():
                        pre_cond_start = None
                        _set_magnet(True)
                        _state("SOFT_CAPTURE")
            else:
                if pre_cond_start is not None:
                    print("\n[SM] 조건 소실 → 타이머 리셋")
                pre_cond_start = None
                _cancel_approval()

        # ═══════════════════════════════════════════════
        # State 2: SOFT_CAPTURE
        # ═══════════════════════════════════════════════
        elif state == "SOFT_CAPTURE":
            _set_magnet(True)
            _set_motor_target(MOTOR_EXTENDED_STEPS)

            if sc_enter_t is None:
                sc_enter_t = now

            # 60s 타임아웃 → PRE_DOCKING (안전)
            if now - sc_enter_t >= 60.0:
                print("\n[SM] SOFT_CAPTURE 60s 타임아웃 → PRE_DOCKING")
                sc_enter_t      = None
                dist_lock_start = None
                _cancel_approval()
                _set_magnet(False)
                _state("PRE_DOCKING")

            elif dist >= 0 and dist <= HARD_LOCK_DIST_MM:
                if dist_lock_start is None:
                    dist_lock_start = now
                    print(f"\n[SM] ToF {int(dist)}mm ≤ {HARD_LOCK_DIST_MM}mm 감지, "
                          f"{HARD_LOCK_HOLD_S}s 카운트...")
                elif now - dist_lock_start >= HARD_LOCK_HOLD_S:
                    _request_approval("HARD_LOCK")
                    if _check_approved():
                        dist_lock_start = None
                        sc_enter_t      = None
                        _state("HARD_LOCK")
            else:
                if dist_lock_start is not None:
                    print("\n[SM] 거리 초과 → 타이머 리셋")
                dist_lock_start = None
                _cancel_approval()

        # ═══════════════════════════════════════════════
        # State 3: HARD_LOCK
        # ═══════════════════════════════════════════════
        elif state == "HARD_LOCK":
            _set_magnet(True)
            _set_motor_target(0)  # 6000 → 0 복귀

            if hl_enter_t is None:
                hl_enter_t = now
                print("\n[SM] HARD_LOCK 진입, 모터 복귀 시작 (6000→0)...")

            # 120s 타임아웃 (안전)
            if now - hl_enter_t >= 120.0:
                print("\n[SM] HARD_LOCK 120s 타임아웃 → PRE_DOCKING")
                hl_enter_t        = None
                motor_zero_done_t = None
                _cancel_approval()
                _set_magnet(False)
                _state("PRE_DOCKING")
            else:
                motor_at_zero = (steps == 0 and tgt == 0)
                if motor_at_zero:
                    if motor_zero_done_t is None:
                        motor_zero_done_t = now
                        print(f"\n[SM] 모터 step 0 도달, {DOCKED_WAIT_S}s 대기...")
                    elif now - motor_zero_done_t >= DOCKED_WAIT_S:
                        _request_approval("DOCKED")
                        if _check_approved():
                            motor_zero_done_t = None
                            hl_enter_t        = None
                            _set_magnet(False)
                            _state("DOCKED")
                else:
                    motor_zero_done_t = None

        # ═══════════════════════════════════════════════
        # State 4: DOCKED
        # ═══════════════════════════════════════════════
        elif state == "DOCKED":
            _set_magnet(False)
            _set_motor_target(0)

            if docked_enter_t is None:
                docked_enter_t = now
                print(f"\n[SM] DOCKED 진입, {CHARGING_WAIT_S}s 후 CHARGING 승인 대기...")
            elif now - docked_enter_t >= CHARGING_WAIT_S:
                _request_approval("CHARGING")
                if _check_approved():
                    docked_enter_t = None
                    _set_servo(SERVO_CHARGING_ANGLE)
                    _state("CHARGING")

        # ═══════════════════════════════════════════════
        # State 5: CHARGING
        # ═══════════════════════════════════════════════
        elif state == "CHARGING":
            _set_magnet(False)
            _set_motor_target(0)
            _set_servo(SERVO_CHARGING_ANGLE)

            if charging_enter_t is None:
                charging_enter_t = now
                print(f"\n[SM] CHARGING 진입 (서보 {SERVO_CHARGING_ANGLE}°), "
                      f"{SEPARATION_CHARGE_WAIT_S}s 후 분리 승인 가능...")
            elif now - charging_enter_t >= SEPARATION_CHARGE_WAIT_S:
                _request_approval("SEPARATION_1")
                if _check_approved():
                    charging_enter_t = None
                    _state("SEPARATION_1")

        # ═══════════════════════════════════════════════
        # State 6: SEPARATION_1
        # ═══════════════════════════════════════════════
        elif state == "SEPARATION_1":
            _set_magnet(True)
            _set_motor_target(0)

            if sep1_enter_t is None:
                sep1_enter_t = now
                _set_servo(SERVO_STOWED_ANGLE)  # 65° → 0° (분리)
                print(f"\n[SM] SEPARATION_1 진입, 서보 0° 회전 ({SERVO_ROTATION_S}s) 후 "
                      f"{SEPARATION_SERVO_WAIT_S}s 대기...")

            # 서보 이동(1s) + 대기(2s) 완료 후 승인 요청
            if now - sep1_enter_t >= (SERVO_ROTATION_S + SEPARATION_SERVO_WAIT_S):
                _request_approval("SEPARATION_2")
                if _check_approved():
                    sep1_enter_t = None
                    _state("SEPARATION_2")

        # ═══════════════════════════════════════════════
        # State 7: SEPARATION_2
        # ═══════════════════════════════════════════════
        elif state == "SEPARATION_2":
            _set_magnet(True)
            _set_servo(SERVO_STOWED_ANGLE)
            _set_motor_target(MOTOR_EXTENDED_STEPS)  # 0 → 6000

            if sep2_enter_t is None:
                sep2_enter_t = now
                print("\n[SM] SEPARATION_2 진입, 모터 6000 steps 연장 중...")

            motor_at_ext = (steps == MOTOR_EXTENDED_STEPS and tgt == MOTOR_EXTENDED_STEPS)
            if motor_at_ext:
                if sep2_motor_done_t is None:
                    sep2_motor_done_t = now
                    print(f"\n[SM] 모터 step {MOTOR_EXTENDED_STEPS} 도달, "
                          f"{SEPARATION_MOTOR_WAIT_S}s 대기...")
                elif now - sep2_motor_done_t >= SEPARATION_MOTOR_WAIT_S:
                    _request_approval("SEPARATION_3")
                    if _check_approved():
                        sep2_enter_t      = None
                        sep2_motor_done_t = None
                        _set_magnet(False)
                        _state("SEPARATION_3")
            else:
                sep2_motor_done_t = None

        # ═══════════════════════════════════════════════
        # State 8: SEPARATION_3  (최종 상태 — RESET만 복귀)
        # ═══════════════════════════════════════════════
        elif state == "SEPARATION_3":
            _set_magnet(False)
            _set_motor_target(MOTOR_EXTENDED_STEPS)
            _set_servo(SERVO_STOWED_ANGLE)
