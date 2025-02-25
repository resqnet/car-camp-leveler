# mpu6050.py

from machine import I2C
from math import sqrt, atan2, degrees
from vector3d import Vector3d

class MPU6050:
    _I2C_ADDR = 0x68

    def __init__(self, i2c, addr=_I2C_ADDR):
        self.i2c = i2c
        self.addr = addr
        self.buf = bytearray(1)
        self._accel = Vector3d(self._accel_callback, self._accel_callback, self._accel_callback)
        self._gyro = Vector3d(self._gyro_callback, self._gyro_callback, self._gyro_callback)
        self.i2c.writeto_mem(self.addr, 0x6b, b'\x00')  # Wake up

    def get_accel_data(self):
        return {
            'x': self._read_word(0x3b) / 16384,
            'y': self._read_word(0x3d) / 16384,
            'z': self._read_word(0x3f) / 16384
        }

    def get_gyro_data(self):
        return {
            'x': self._read_word(0x43) / 131,
            'y': self._read_word(0x45) / 131,
            'z': self._read_word(0x47) / 131
        }

    def _accel_callback(self):
        self._accel._ivector[0] = self._read_word(0x3b) / 16384
        self._accel._ivector[1] = self._read_word(0x3d) / 16384
        self._accel._ivector[2] = self._read_word(0x3f) / 16384

    def _gyro_callback(self):
        self._gyro._ivector[0] = self._read_word(0x43) / 131
        self._gyro._ivector[1] = self._read_word(0x45) / 131
        self._gyro._ivector[2] = self._read_word(0x47) / 131

    def _read_word(self, mem_addr):
        hi = self.i2c.readfrom_mem(self.addr, mem_addr, 1)
        lo = self.i2c.readfrom_mem(self.addr, mem_addr + 1, 1)
        return (hi[0] << 8) + lo[0] if hi[0] < 0x80 else -(((hi[0] ^ 0xff) << 8) | (lo[0] ^ 0xff) + 1)
