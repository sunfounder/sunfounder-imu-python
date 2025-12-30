from sunfounder_imu.qmc6310 import QMC6310
import time 

mag = QMC6310()
mag.set_calibration(
    offsets=[0.04, 0.09, 0.03],
    scales=[1.03, 1.01, 0.96])

while True:
    mag_data, azimuth = mag.read()
    print(f"Magnetometer: ")
    print(f"  x: {mag_data.x:.2f} uT")
    print(f"  y: {mag_data.y:.2f} uT")
    print(f"  z: {mag_data.z:.2f} uT")
    print(f"  azimuth: {azimuth:.2f} degrees")
    time.sleep(0.1)

