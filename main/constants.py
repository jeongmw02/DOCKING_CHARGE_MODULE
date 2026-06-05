# constants.py — 로컬 상수 (config.py와 별도)

# ── MG995 서보 ────────────────────────────────────────────────
SERVO_CHARGING_PIN   = 13      # GPIO13 (PWM1)
SERVO_FREQ_HZ        = 50      # 표준 RC 서보 50Hz
SERVO_CHARGING_ANGLE = 65      # 충전 위치 (deg)
SERVO_STOWED_ANGLE      = 0    # 대기 위치 (deg)
SERVO_SEPARATION_ANGLE = 130  # 분리 위치 — 충전(65°)의 반대 방향 65° (deg)
SERVO_ROTATION_S     = 1.0     # 서보 이동 완료 대기 시간 (s)

# ── 타이머 상수 ─────────────────────────────────────────────
CHARGING_WAIT_S          = 2.0   # DOCKED → CHARGING 대기 (s)
SEPARATION_CHARGE_WAIT_S = 5.0   # CHARGING 후 분리 가능 대기 (s)
DOCKED_WAIT_S            = 2.0   # 모터 step 0 후 DOCKED 전환 대기 (s)
SEPARATION_SERVO_WAIT_S  = 2.0   # 서보 완료 후 SEPARATION_2 대기 (s)
SEPARATION_MOTOR_WAIT_S  = 2.0   # 모터 6000 후 SEPARATION_3 대기 (s)

# ── 모터 스텝 & 속도 ─────────────────────────────────────────
MOTOR_EXTENDED_STEPS = 6000   # 완전 연장 위치 (PRE_DOCKING / SEPARATION)
# 도킹 시: 11000 → 0,  분리 시: 0 → 11000

MOTOR_MIN_DELAY   = 0.003   # 최고 속도 (s/step) — 공진 회피
MOTOR_MAX_DELAY   = 0.008   # 최저 속도 — 가감속 구간 (s/step)
MOTOR_ACCEL_STEPS = 500     # 가감속 구간 스텝 수 (부드러운 시작/정지)

# ── 도킹 조건 ────────────────────────────────────────────────
SOFT_CAPTURE_DIST_MM = 500   # PRE_DOCKING 전이 ToF 임계값 (mm)
SOFT_CAPTURE_HOLD_S  = 0.5   # PRE_DOCKING 전이 지속 시간 (s) — ToF & ArUco 동시
HARD_LOCK_DIST_MM    = 280    # SOFT_CAPTURE → HARD_LOCK ToF 임계값 (mm)
HARD_LOCK_HOLD_S     = 5.0   # SOFT_CAPTURE → HARD_LOCK 지속 시간 (s)


