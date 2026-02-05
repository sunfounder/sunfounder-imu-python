from sunfounder_imu.sensors.qmi8658b import QMI8658B
import time

qmi8658b = QMI8658B()

while True:
    accel = qmi8658b.read_raw_accel()
    print(accel)
    time.sleep(1)
