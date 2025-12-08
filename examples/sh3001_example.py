from sunfounder_imu.sh3001 import SH3001
import time

imu = SH3001()
imu.set_gyro_offset([-262.44, -307.14, -64.22])
imu.set_accel_offset([-540.5, -65.5, 225.5])

while True:
    accData, gyroData, temperature = imu.read()
    print(f"Acceleration: ")
    print(f"  x: {accData.x:.2f} m/s^2")
    print(f"  y: {accData.y:.2f} m/s^2")
    print(f"  z: {accData.z:.2f} m/s^2")
    print(f"Gyroscope: ")
    print(f"  x: {gyroData.x:.2f} d/s")
    print(f"  y: {gyroData.y:.2f} d/s")
    print(f"  z: {gyroData.z:.2f} d/s")
    time.sleep(1)
