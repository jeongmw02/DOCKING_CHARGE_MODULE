# hardware/tof_sensor.py
# VL53L0X ToF 거리 센서 드라이버 (단일 센서)
#
# 배선:
#   VCC  → 3.3V (Pin 1)
#   GND  → GND  (Pin 6)
#   SDA  → GPIO2 (Pin 3)
#   SCL  → GPIO3 (Pin 5)
#
# 의존 라이브러리:
#   pip install adafruit-circuitpython-vl53l0x

import time
import board
import busio
import adafruit_vl53l0x
from utils import get_logger

log = get_logger("ToF")

VALID_MIN_MM = 10
VALID_MAX_MM = 2000


class SensorReading:
    """단일 센서 측정값."""
    def __init__(self, distance_mm: float, valid: bool = True):
        self.distance_mm = distance_mm
        self.valid       = valid
        self.timestamp   = time.time()


class ToFSensor:
    """VL53L0X 단일 센서 드라이버."""

    def __init__(self):
        self._sensor = None
        self._i2c    = None

    def setup(self):
        self._i2c   = busio.I2C(board.SCL, board.SDA)
        self._sensor = adafruit_vl53l0x.VL53L0X(self._i2c)
        log.info("VL53L0X 초기화 완료")

    def read(self) -> SensorReading:
        """거리 측정. 유효 범위 벗어나면 valid=False."""
        try:
            dist_mm = self._sensor.range
            valid   = VALID_MIN_MM <= dist_mm <= VALID_MAX_MM
            if not valid:
                log.debug(f"범위 초과: {dist_mm}mm")
            return SensorReading(distance_mm=dist_mm, valid=valid)
        except Exception as e:
            log.error(f"센서 읽기 오류: {e}")
            return SensorReading(distance_mm=9999, valid=False)

    def cleanup(self):
        if self._i2c:
            self._i2c.deinit()
        log.info("ToF 정리 완료")

