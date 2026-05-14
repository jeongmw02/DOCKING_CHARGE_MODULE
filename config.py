# config.py
# CubeSat 도킹·충전 모듈 - 전역 설정
# GPIO 핀 번호, 물리 파라미터, 상태 머신 임계값을 한 곳에서 관리한다.

# ── GPIO 핀 번호 (BCM 모드) ──────────────────────────────
# ToF 센서 (I2C, XSHUT 핀)
TOF_I2C_BUS       = 1
TOF_SENSOR_COUNT  = 3
TOF_XSHUT_PINS    = [17, 27, 22]
TOF_I2C_ADDRESSES = [0x30, 0x31, 0x32]

# 전자석
ELECTROMAGNET_PIN = 18

# 스테퍼 모터 (NEMA17 + A4988)
STEPPER_STEP_PIN      = 12   # 실제 배선 핀
STEPPER_DIR_PIN       = 17   # 실제 배선 핀
STEPPER_EN_PIN        = 25
STEPPER_STEPS_PER_REV = 200
STEPPER_MICROSTEP     = 8
STEPPER_LEAD_MM       = 8

# 서보 (솔라 패널)
SERVO_PANEL_A_PIN = 12
SERVO_PANEL_B_PIN = 13
SERVO_FREQ_HZ     = 50

# Pogo Pin
POGO_DETECT_PIN    = 16
POGO_CHARGE_EN_PIN = 20

# ── 상태 머신 임계값 (main_ver1.py 기준) ────────────────
SOFT_CAPTURE_DIST_MM    = 300   # mm - ToF ≤ 이 값 3s 지속 → SOFT_CAPTURE
HARD_LOCK_DIST_MM       = 150    # mm - ToF ≤ 이 값 5s 지속 → HARD_LOCK
ALIGNMENT_TOLERANCE_MM  = 5

# ── 상태 머신 타이머 ─────────────────────────────────────
SOFT_CAPTURE_HOLD_S     = 3.0   # 초 - SOFT_CAPTURE 진입 조건 지속 시간
HARD_LOCK_HOLD_S        = 5.0   # 초 - HARD_LOCK 진입 조건 지속 시간
DOCKED_WAIT_S           = 2.0   # 초 - 모터 step0 도달 후 DOCKED 전환 대기
MARKER_LOST_TIMEOUT_S   = 1.5   # 초 - TARGET_LOCK 마커 소실 유예 시간

# ── 모터 ─────────────────────────────────────────────────
MOTOR_TARGET_STEPS      = 3000  # 도킹 시 전진 스텝
MOTOR_STEP_DELAY_S      = 0.005 # 스텝당 딜레이 (5ms, 실제 작동 확인값)

# ── 속도 ────────────────────────────────────────────────
APPROACH_SPEED_MM_S = 10.0
FINE_SPEED_MM_S     = 2.0

# ── 타임아웃 (초) ────────────────────────────────────────
APPROACH_TIMEOUT_S     = 30
SOFT_CAPTURE_TIMEOUT_S = 10
HARD_LOCK_TIMEOUT_S    = 5

# ── 서보 각도 ────────────────────────────────────────────
SERVO_MIN_PULSE_MS       = 0.5
SERVO_MAX_PULSE_MS       = 2.5
SOLAR_PANEL_STOWED_DEG   = 0
SOLAR_PANEL_DEPLOYED_DEG = 90

# ── 전자석 ──────────────────────────────────────────────
ELECTROMAGNET_PULL_DUTY = 100   # % - 초기 흡착 풀 파워
ELECTROMAGNET_HOLD_DUTY = 40    # % - 유지 절전 모드
ELECTROMAGNET_PULL_MS   = 500   # ms - 풀 파워 유지 시간

# ── 로깅 ────────────────────────────────────────────────
LOG_LEVEL   = "DEBUG"
LOG_FILE    = "docking.log"
LOG_TO_FILE = True
