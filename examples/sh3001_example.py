
from sunfounder_imu.sh3001 import SH3001
import time

imu = SH3001()

while True:
    accData, gyroData, temperature = imu.read()
    print(f"Acceleration: ")
    print(f"  x: {accData.x:.2f}")
    print(f"  y: {accData.y:.2f}")
    print(f"  z: {accData.z:.2f}")
    print(f"Gyroscope: ")
    print(f"  x: {gyroData.x:.2f}")
    print(f"  y: {gyroData.y:.2f}")
    print(f"  z: {gyroData.z:.2f}")
    print(f"Temperature: {temperature:.2f} °C")
    time.sleep(1)
