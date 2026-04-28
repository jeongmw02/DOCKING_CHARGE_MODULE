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

# 스테퍼 모터 (NEMA17 + DRV8825)
STEPPER_STEP_PIN      = 23
STEPPER_DIR_PIN       = 24
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

# ── 상태 머신 임계값 ────────────────────────────────────
APPROACH_START_DIST_MM = 300
SOFT_CAPTURE_DIST_MM   = 50
HARD_LOCK_CONFIRM_MM   = 10
ALIGNMENT_TOLERANCE_MM = 5

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
