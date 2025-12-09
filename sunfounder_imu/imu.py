
import numpy as np
from typing import Optional
import math

from ._i2c import I2C
from .spl06_001 import SPL06_001
from .qmc6310 import QMC6310
from .sh3001 import SH3001
from .data_type import AccelDate, GyroDate, MagDate

class IMU:
    BAROMETER_ADDRESS_MAP = {
        0x76: SPL06_001,
        0x77: SPL06_001,
    }

    MAGNETOMETER_ADDRESS_MAP = {
        0x1C: QMC6310,
        0x3C: QMC6310,
    }

    ACCEL_GYRO_ADDRESS_MAP = {
        0x36: SH3001,
        0x37: SH3001,
    }

    def __init__(self, kp: float=0.5, ki: float=0.0, sample_freq: float=100.0):
        self._temperature = None
        self.acceleration = AccelDate(0, 0, 0)
        self.gyroscrope = GyroDate(0, 0, 0)
        self.magenatic = MagDate(0, 0, 0)

        self.barometer_sensor = None
        self.accel_gyro_sensor = None
        self.magenatic_sensor = None

        self.temperature = None
        self.pressure = None
        self.altitude = None

        self.kp = kp
        self.ki = ki
        self.dt = 1.0 / sample_freq
        # 四元数（表示姿态，比欧拉角更稳定，无万向节锁）
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.integral_error = np.array([0.0, 0.0, 0.0])

        addresses = I2C.scan()
        for address in addresses:
            if address in self.BAROMETER_ADDRESS_MAP:
                self.barometer_sensor = self.BAROMETER_ADDRESS_MAP[address]()
            if address in self.ACCEL_GYRO_ADDRESS_MAP:
                self.accel_gyro_sensor = self.ACCEL_GYRO_ADDRESS_MAP[address]()
            if address in self.MAGNETOMETER_ADDRESS_MAP:
                self.magenatic_sensor = self.MAGNETOMETER_ADDRESS_MAP[address]()

    def read_raw(self):
        """ Get all measurements in one call
            
        Returns:
            dict: Dictionary containing temperature, pressure, and altitude
        """
        if self.barometer_sensor is not None:
            self.temperature, self.pressure, self.altitude = self.barometer_sensor.read()
        if self.accel_gyro_sensor is not None:
            self.acceleration, self.gyroscrope = self.accel_gyro_sensor.read()
        if self.magenatic_sensor is not None:
            self.magenatic, self.azimuth = self.magenatic_sensor.read()
        
        return {
            "acceleration": self.acceleration,
            "gyroscrope": self.gyroscrope,
            "magenatic": self.magenatic,
            "azimuth": self.azimuth,
            "temperature": self.temperature,
            "pressure": self.pressure,
            "altitude": self.altitude,
        }

    def get_euler_angles(self, acc: list, gyro: list, mag: list) -> list:
        """更新姿态，返回欧拉角（角度）"""
        q = self.q
        ax, ay, az = acc
        gx, gy, gz = gyro
        mx, my, mz = mag

        # 归一化加速度计和磁力计
        acc_norm = np.linalg.norm(acc)
        if acc_norm == 0:
            return self.quat_to_euler(q)
        acc = acc / acc_norm

        mag_norm = np.linalg.norm(mag)
        if mag_norm == 0:
            return self.quat_to_euler(q)
        mag = mag / mag_norm

        # -------------------------- 计算期望方向（重力+磁场） --------------------------
        # 从四元数推导重力向量（期望加速度方向）
        vx = 2 * (q[1]*q[3] - q[0]*q[2])
        vy = 2 * (q[0]*q[1] + q[2]*q[3])
        vz = q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2

        # 磁场向量修正（消除倾斜影响）
        hx = mx * (q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2) + my * 2*(q[1]*q[2] - q[0]*q[3]) + mz * 2*(q[1]*q[3] + q[0]*q[2])
        hy = mx * 2*(q[1]*q[2] + q[0]*q[3]) + my * (q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2) + mz * 2*(q[2]*q[3] - q[0]*q[1])
        bx = math.sqrt(hx**2 + hy**2)
        bz = mx * 2*(q[1]*q[3] - q[0]*q[2]) + my * 2*(q[2]*q[3] + q[0]*q[1]) + mz * (q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2)
        
        # 期望磁场方向
        wx = bx
        wy = 0
        wz = bz

        # -------------------------- 计算误差（实际 vs 期望） --------------------------
        # 加速度误差（叉乘：实际重力 vs 期望重力）
        acc_error = np.cross(acc, np.array([vx, vy, vz]))
        # 磁场误差（叉乘：实际磁场 vs 期望磁场）
        mag_error = np.cross(np.array([hx, hy, bz]), np.array([wx, wy, wz]))
        # 总误差
        error = acc_error + mag_error

        # -------------------------- 误差反馈修正陀螺仪 --------------------------
        self.integral_error += error * self.dt  # 积分项
        gyro_corrected = gyro + self.kp * error + self.ki * self.integral_error

        # -------------------------- 更新四元数（陀螺仪积分） --------------------------
        q_dot = 0.5 * np.array([
            -q[1]*gyro_corrected[0] - q[2]*gyro_corrected[1] - q[3]*gyro_corrected[2],
            q[0]*gyro_corrected[0] + q[2]*gyro_corrected[2] - q[3]*gyro_corrected[1],
            q[0]*gyro_corrected[1] - q[1]*gyro_corrected[2] + q[3]*gyro_corrected[0],
            q[0]*gyro_corrected[2] + q[1]*gyro_corrected[1] - q[2]*gyro_corrected[0]
        ])
        self.q = q + q_dot * self.dt
        self.q = self.q / np.linalg.norm(self.q)  # 归一化四元数

        # 四元数转欧拉角
        return self.quat_to_euler(self.q)

    def quat_to_euler(self, q):
        """四元数转欧拉角（Roll/Pitch/Yaw，角度）"""
        w, x, y, z = q
        # Roll (x轴)
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        # Pitch (y轴)
        pitch = math.asin(2*(w*y - z*x))
        # Yaw (z轴)
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        # 转角度并归一化
        roll = math.degrees(roll) % 360
        pitch = math.degrees(pitch) % 360
        yaw = math.degrees(yaw) % 360
        return roll, pitch, yaw

    def read(self) -> dict:
        """读取所有传感器数据并更新姿态"""
        data = self.read_raw()
        roll, pitch, yaw = self.get_euler_angles(
            data["acceleration"].tolist(),
            data["gyroscrope"].tolist(),
            data["magenatic"].tolist()
        )
        data["roll"] = roll
        data["pitch"] = pitch
        data["yaw"] = yaw
        return data