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
    sensors = imu.find_sensor()
    if len(sensors) == 0:
        print("No sensor found.")
        return
    for address, class_type in sensors.items():
        name = class_type.__name__
        print(f"Found sensor {name} at address 0x{address:02X}")
    sleep(3)
    
    try:
        while True:
            data = imu.read()
            
            print(f"Altitude={data['altitude']:7.2f} m")
            print(f"Azimuth={data['azimuth']:7.2f} °")
            print(f"Acceleration:")
            print(f"  X={data['acceleration'].x:7.2f} m/s²")
            print(f"  Y={data['acceleration'].y:7.2f} m/s²")
            print(f"  Z={data['acceleration'].z:7.2f} m/s²")
            print(f"Gyroscrope:")
            print(f"  X={data['gyroscrope'].x:7.2f} °/s")
            print(f"  Y={data['gyroscrope'].y:7.2f} °/s")
            print(f"  Z={data['gyroscrope'].z:7.2f} °/s")
            print(f"Magenatic:")
            print(f"  X={data['magenatic'].x:7.2f} μT")
            print(f"  Y={data['magenatic'].y:7.2f} μT")
            print(f"  Z={data['magenatic'].z:7.2f} μT")
            print(f"Temperature={data['temperature']:7.2f} °C")
            print(f"Pressure={data['pressure']:7.2f} hPa")

            sleep(0.5)

    except KeyboardInterrupt:
        print("\nExiting program...")


if __name__ == "__main__":
    main()
