#!/usr/bin/env python3
"""
Read data from all three sensors on the 10 Axis IMU module:
- SPL06_001: temperature, pressure, altitude
- SH3001: temperature, accelerometer, gyroscope
- QMC6310: compass heading

Gracefully exits when Ctrl+C is pressed.
"""

from time import sleep

# Import Sunfounder sensor modules
from sunfounder_imu import IMU

def main():
    imu = IMU()
    
    try:
        while True:
            data = imu.read()
            print("\n")
            if "accel_x" in data and "accel_y" in data and "accel_z" in data:
                print(f"Acceleration:")
                print(f"  X={data['accel_x']:7.2f} m/s²")
                print(f"  Y={data['accel_y']:7.2f} m/s²")
                print(f"  Z={data['accel_z']:7.2f} m/s²")
            if "gyro_x" in data and "gyro_y" in data and "gyro_z" in data:
                print(f"Gyroscrope:")
                print(f"  X={data['gyro_x']:7.2f} °/s")
                print(f"  Y={data['gyro_y']:7.2f} °/s")
                print(f"  Z={data['gyro_z']:7.2f} °/s")
            if "mag_x" in data and "mag_y" in data and "mag_z" in data:
                print(f"Magenatic:")
                print(f"  X={data['mag_x']:7.2f} gauss")
                print(f"  Y={data['mag_y']:7.2f} gauss")
                print(f"  Z={data['mag_z']:7.2f} gauss")
            if "altitude" in data:
                print(f"    Altitude = {data['altitude']:.2f} m")
            if "azimuth" in data:
                print(f"     Azimuth = {data['azimuth']:.2f} °")
            if "temperature" in data:
                print(f" Temperature = {data['temperature']:.2f} °C")
            if "pressure" in data:
                print(f"    Pressure = {data['pressure']:.2f} hPa")

            sleep(0.5)

    except KeyboardInterrupt:
        print("\nExiting program...")


if __name__ == "__main__":
    main()
