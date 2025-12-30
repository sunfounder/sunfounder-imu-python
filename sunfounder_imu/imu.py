
import numpy as np
import math
from ._config import Config
from ._base import _Base
import os

from ._i2c import I2C
from .sensors import get_baro_sensor, get_accel_gyro_sensor, get_mag_sensor

USER_HOME = os.path.expanduser("~")

class IMU(_Base):
    DEFAULT_CONFIG_FILE = os.path.join(USER_HOME, ".config", "sunfounder-imu-config.json")

    def __init__(self,
            *args,
            kp: float=0.5,
            ki: float=0.0,
            sample_freq: float=100.0,
            config_file: str = DEFAULT_CONFIG_FILE,
            **kwargs
            ):

        super().__init__(*args, **kwargs)
        self.config_file = config_file
        self.config = Config(config_file)

        self.baro = None
        self.accel_gyro = None
        self.mag = None

        self.kp = kp
        self.ki = ki
        self.dt = 1.0 / sample_freq
        # Quaternion (w, x, y, z) to represent attitude, more stable than Euler angles, no gimbal lock
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.integral_error = np.array([0.0, 0.0, 0.0])

        addresses = I2C.scan()
        self.mag = get_mag_sensor(addresses)
        self.baro = get_baro_sensor(addresses)
    
        # Setup accel_gyro sensor
        self.accel_gyro = get_accel_gyro_sensor(addresses)
        if self.accel_gyro is not None:
            accel_offset = self.config.get(f"acc_offset", default=[0, 0, 0])
            accel_scale = self.config.get(f"acc_scale", default=[1.0, 1.0, 1.0])
            gyro_offset = self.config.get(f"gyro_offset", default=[0, 0, 0])
            self.accel_gyro.set_calibration_data(accel_offset, accel_scale, gyro_offset)
        else:
            self.log.warning("No accelerometer-gyroscope sensor found.")
        
        # Setup magnetometer sensor
        if self.mag is not None:
            mag_offset = self.config.get(f"mag_offset", default=[0, 0, 0])
            mag_scale = self.config.get(f"mag_scale", default=[1.0, 1.0, 1.0])
            self.mag.set_calibration_data(mag_offset, mag_scale)
        else:
            self.log.warning("No magnetometer sensor found.")

        # Setup barometer sensor
        if self.baro is not None:
            sea_level_pressure = self.config.get("barometer_sea_level_pressure", self.baro.P0)
            offset = self.config.get(f"barometer_offset", 0.0)
            self.baro.set_calibration_data(sea_level_pressure, offset)
        else:
            self.log.warning("No barometer sensor found.")

        if self.accel_gyro is None and self.mag is None and self.baro is None:
            raise ValueError("No sensor found.")

    def read_raw(self):
        """ Get all measurements in one call
            
        Returns:
            dict: Dictionary containing temperature, pressure, and altitude
        """
        result = {}
        if self.baro is not None:
            self.temperature, self.pressure, self.altitude = self.baro.read()
            result["temperature"] = self.temperature
            result["pressure"] = self.pressure
            result["altitude"] = self.altitude
        if self.accel_gyro is not None:
            self.accel_data, self.gyro_data, _ = self.accel_gyro.read()
            result["accel_x"] = self.accel_data[0]
            result["accel_y"] = self.accel_data[1]
            result["accel_z"] = self.accel_data[2]
            result["gyro_x"] = self.gyro_data[0]
            result["gyro_y"] = self.gyro_data[1]
            result["gyro_z"] = self.gyro_data[2]
        if self.mag is not None:
            self.mag_data, self.azimuth = self.mag.read()
            result["mag_x"] = self.mag_data[0]
            result["mag_y"] = self.mag_data[1]
            result["mag_z"] = self.mag_data[2]
            result["azimuth"] = self.azimuth
        
        return result

    def get_euler_angles(self, acc: list, gyro: list, mag: list) -> list:
        """Update attitude and return Euler angles (degrees)

        Args:
            acc (list): Acceleration data [ax, ay, az]
            gyro (list): Gyroscope data [gx, gy, gz]
            mag (list): Magnetometer data [mx, my, mz]

        Returns:
            list: Euler angles [roll, pitch, yaw] in degrees
        """
        q = self.q
        ax, ay, az = acc
        gx, gy, gz = gyro
        mx, my, mz = mag

        # Normalize acceleration and magnetometer vectors
        acc_norm = np.linalg.norm(acc)
        if acc_norm == 0:
            return self.quat_to_euler(q)
        acc = acc / acc_norm

        mag_norm = np.linalg.norm(mag)
        if mag_norm == 0:
            return self.quat_to_euler(q)
        mag = mag / mag_norm

        # -------------------------- Calculate expected direction (gravity + magnetometer) --------------------------
        # Get expected gravity direction (from quaternion)
        vx = 2 * (q[1]*q[3] - q[0]*q[2])
        vy = 2 * (q[0]*q[1] + q[2]*q[3])
        vz = q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2

        # Get expected magnetometer direction (from quaternion)
        hx = mx * (q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2) + my * 2*(q[1]*q[2] - q[0]*q[3]) + mz * 2*(q[1]*q[3] + q[0]*q[2])
        hy = mx * 2*(q[1]*q[2] + q[0]*q[3]) + my * (q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2) + mz * 2*(q[2]*q[3] - q[0]*q[1])
        bx = math.sqrt(hx**2 + hy**2)
        bz = mx * 2*(q[1]*q[3] - q[0]*q[2]) + my * 2*(q[2]*q[3] + q[0]*q[1]) + mz * (q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2)
        
        # Get expected magnetometer direction (from quaternion)
        wx = bx
        wy = 0
        wz = bz

        # -------------------------- Calculate error (actual vs expected) --------------------------
        # Acceleration error (cross product: actual gravity vs expected gravity)
        acc_error = np.cross(acc, np.array([vx, vy, vz]))
        # Magnetometer error (cross product: actual magnetometer vs expected magnetometer)
        mag_error = np.cross(np.array([hx, hy, bz]), np.array([wx, wy, wz]))
        # Total error
        error = acc_error + mag_error

        # -------------------------- Error feedback correction to gyroscope --------------------------
        self.integral_error += error * self.dt  # Integral term
        gyro_corrected = gyro + self.kp * error + self.ki * self.integral_error

        # -------------------------- Update quaternion (gyroscope integration) --------------------------
        q_dot = 0.5 * np.array([
            -q[1]*gyro_corrected[0] - q[2]*gyro_corrected[1] - q[3]*gyro_corrected[2],
            q[0]*gyro_corrected[0] + q[2]*gyro_corrected[2] - q[3]*gyro_corrected[1],
            q[0]*gyro_corrected[1] - q[1]*gyro_corrected[2] + q[3]*gyro_corrected[0],
            q[0]*gyro_corrected[2] + q[1]*gyro_corrected[1] - q[2]*gyro_corrected[0]
        ])
        self.q = q + q_dot * self.dt
        self.q = self.q / np.linalg.norm(self.q)  # Normalize quaternion
        # Convert quaternion to Euler angles (Roll/Pitch/Yaw, degrees)
        return self.quat_to_euler(self.q)

    def quat_to_euler(self, q):
        """Convert quaternion to Euler angles (Roll/Pitch/Yaw, degrees)"""
        w, x, y, z = q
        # Roll (x-axis)
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        # Pitch (y-axis)
        pitch = math.asin(2*(w*y - z*x))
        # Yaw (z-axis)
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        # Convert to degrees and normalize to [0, 360)
        roll = math.degrees(roll) % 360
        pitch = math.degrees(pitch) % 360
        yaw = math.degrees(yaw) % 360
        return roll, pitch, yaw

    def read(self) -> dict:
        """Read all sensor data and update attitude"""
        data = self.read_raw()
        roll, pitch, yaw = self.get_euler_angles(
            [data["accel_x"], data["accel_y"], data["accel_z"]],
            [data["gyro_x"], data["gyro_y"], data["gyro_z"]],
            [data["mag_x"], data["mag_y"], data["mag_z"]]
        )
        data["roll"] = roll
        data["pitch"] = pitch
        data["yaw"] = yaw
        return data

    def calibrate_accel_prepare(self) -> None:
        """Calibrate the accelerometer, prepare calibration data."""
        self.accel_gyro.calibrate_accel_prepare()
    
    def calibrate_accel_step(self) -> list:
        """Calibrate the accelerometer, return current calibration data."""
        return self.accel_gyro.calibrate_accel_step()

    def calibrate_accel_finish(self) -> list:
        """Calibrate the accelerometer, return current calibration data."""
        acc_offset, acc_scale, acc_max, acc_min = self.accel_gyro.calibrate_accel_finish()
        self.accel_gyro.set_accel_offset(acc_offset)
        self.accel_gyro.set_accel_scale(acc_scale)
        self.config.set(f"accel_offset", acc_offset)
        self.config.set(f"accel_scale", acc_scale)
        return acc_offset, acc_scale, acc_max, acc_min

    def calibrate_gyro_prepare(self) -> None:
        """Calibrate the gyroscope, prepare calibration data."""
        self.accel_gyro.calibrate_gyro_prepare()
    
    def calibrate_gyro_step(self) -> list:
        """Calibrate the gyroscope, return current calibration data."""
        return self.accel_gyro.calibrate_gyro_step()

    def calibrate_gyro_finish(self) -> list:
        """Calibrate the gyroscope, return current calibration data."""
        gyro_offset = self.accel_gyro.calibrate_gyro_finish()
        self.accel_gyro.set_gyro_offset(gyro_offset)
        self.config.set(f"gyro_offset", gyro_offset)
        return gyro_offset

    def calibrate_mag_prepare(self) -> None:
        """Calibrate the magnetometer, prepare calibration data."""
        self.mag.calibrate_prepare()
    
    def calibrate_mag_step(self) -> list:
        """Calibrate the magnetometer, return current calibration data."""
        return self.mag.calibrate_step()

    def calibrate_mag_finish(self) -> list:
        """Calibrate the magnetometer, return current calibration data."""
        mag_offset, mag_scale = self.mag.calibrate_finish()
        self.mag.set_calibration_data(mag_offset, mag_scale)
        self.config.set(f"mag_offset", mag_offset)
        self.config.set(f"mag_scale", mag_scale)
        return mag_offset, mag_scale
