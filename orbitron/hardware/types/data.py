

from typing import Tuple


class IMUData:
    def __init__(self, timestamp: float, gyro: Tuple[float, float, float], accel: Tuple[float, float, float]):
        self.timestamp = timestamp
        self.gyro = gyro
        self.accel = accel
