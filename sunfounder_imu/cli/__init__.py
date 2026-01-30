import argparse

CHOICES = [
    "calibrate",
    "clear-calibration",
    "scan",
]
parser = argparse.ArgumentParser(description="SunFounder IMU CLI")
parser.add_argument("command", choices=CHOICES, help="Command to execute")

def main():
    args = parser.parse_args()

    if args.command == "calibrate":
        from .calibrate import main
        main()
    elif args.command == "clear-calibration":
        from ..constants import DEFAULT_CONFIG_FILE
        import os
        if os.path.exists(DEFAULT_CONFIG_FILE):
            os.remove(DEFAULT_CONFIG_FILE)
            print(f"Calibration file {DEFAULT_CONFIG_FILE} cleared")
        else:
            print(f"Calibration file {DEFAULT_CONFIG_FILE} does not exist")
    elif args.command == "scan":
        
        from sunfounder_imu import IMU

        imu = IMU()

        if imu.accel_gyro is not None:
            print("Accel & Gyro sensor:")
            print(f"  name: {imu.accel_gyro.__class__.__name__}")
            print(f"  address: 0x{imu.accel_gyro.address:02x}")
        if imu.mag is not None:
            print("Mag sensor:")
            print(f"  name: {imu.mag.__class__.__name__}")
            print(f"  address: 0x{imu.mag.address:02x}")
        if imu.baro is not None:
            print("Baro sensor:")
            print(f"  name: {imu.baro.__class__.__name__}")
            print(f"  address: 0x{imu.baro.address:02x}")
