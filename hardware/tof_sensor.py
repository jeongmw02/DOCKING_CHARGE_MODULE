# hardware/tof_sensor.py
# VL53L0X / VL53L1X ToF 거리 센서 드라이버
#
# 복수 센서 전략:
#   XSHUT 핀을 순차 HIGH → 각 센서에 고유 I2C 주소 할당
#
# 의존 라이브러리:
#   pip install VL53L1X smbus2

import time
import config
from utils import get_logger

log = get_logger("ToF")


class SensorReading:
    """단일 센서 측정값."""
    def __init__(self, index: int, distance_mm: float, valid: bool = True):
        self.index       = index        # 0=전방, 1=좌, 2=우
        self.distance_mm = distance_mm
        self.valid       = valid
        self.timestamp   = time.time()


class ToFSensorArray:
    """복수 VL53L1X 센서 배열 관리."""

    def __init__(self):
        self._sensors = []
        self._ready   = False

    def setup(self):
        """XSHUT 핀 순차 제어로 센서별 I2C 주소 할당."""
        # TODO: GPIO.setup(XSHUT_PIN, OUT, LOW) × N
        # TODO: 센서마다 XSHUT HIGH → VL53L1X.open() → change_address()
        # TODO: start_ranging(mode=3)
        self._ready = True
        log.info("ToF 배열 초기화 완료")

    def read_all(self) -> list[SensorReading]:
        """전체 센서 거리 측정."""
        # TODO: 각 sensor.get_distance() 호출 → SensorReading 리스트 반환
        # TODO: 유효 범위(10~4000mm) 체크
        return []

    def read_front(self) -> SensorReading:
        """전방 센서(index 0) 단독 측정."""
        # TODO: read_all()[0] 반환
        return SensorReading(index=0, distance_mm=9999, valid=False)

    def get_alignment_offset_mm(self) -> float:
        """좌우 센서 거리 차이 (양수=오른쪽 치우침)."""
        # TODO: readings[2].distance_mm - readings[1].distance_mm
        return 0.0

    def cleanup(self):
        # TODO: 각 sensor.stop_ranging() / sensor.close()
        log.info("ToF 정리 완료")
