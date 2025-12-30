
from sunfounder_imu.spl06_001 import SPL06_001
import time

barometer = SPL06_001(log_level="DEBUG")

while True:
    temperature, pressure, altitude = barometer.read()
    print(f"Pressure: {pressure:.2f} hPa, Temperature: {temperature:.2f} °C, Altitude: {altitude:.2f} m")
    time.sleep(1)
