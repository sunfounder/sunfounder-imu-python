#!/usr/bin/env python3
"""
Read data from all three sensors on the 10 Axis IMU module:
- SPL06_001: temperature, pressure, altitude
- SH3001: temperature, accelerometer, gyroscope
- QMC6310: compass heading

Gracefully exits when Ctrl+C is pressed.
"""

from time import sleep
import math

# Import Sunfounder sensor modules
from sunfounder_imu import IMU

def main():
    imu = IMU()

    try:
        while True:
            imu.read()
            
            print(f"Altitude={imu.altitude:7.2f} m")
            sleep(0.5)

    except KeyboardInterrupt:
        print("\nExiting program...")


if __name__ == "__main__":
    main()
