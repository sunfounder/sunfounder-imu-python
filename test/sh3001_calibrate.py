from sunfounder_imu.sh3001 import SH3001
import time

imu = SH3001()

def calibrate_gyro():
    """ Calibrate the gyroscope.
    """
    print("Let's calibrate the gyroscope. set the device down and DO NOT move it until the calibration is complete.")
    input("When you are ready, press any key to continue.")
    gyro_offsets = imu.calibrate_gyro()
    print("Gyroscope calibration complete. data saved.")
    print(f"Gyroscope offsets: {gyro_offsets}")

def calibrate_accel(seconds: int=10):
    """ Calibrate the accelerometer.
    """
    print(f"Let's calibrate the accelerometer. When the calibration starts, spin the device in all directions. after {seconds} seconds, calibration will finish.")
    input("When you are ready, press any key to continue.")
    imu.calibrate_accel_prepare()
    start_time = time.time()
    while time.time() - start_time < seconds:
        data = imu.calibrate_accel_step()
        delta_time = time.time() - start_time
        print(f"({delta_time:.2f}/{seconds:.2f}), Acceleration: {data}                 ", end="\r")
        time.sleep(0.001)
    accel_offsets, accel_max, accel_min = imu.calibrate_accel_finish()
    print("Accelerometer calibration complete. data saved.")
    print(f"Accelerometer offsets: {accel_offsets}")
    print(f"Accelerometer max: {accel_max}")
    print(f"Accelerometer min: {accel_min}")

def main():
    while True:
        print("\n\nSH3001 calibration")
        print("Choose the device to calibrate:")
        print("1. Gyroscope")
        print("2. Accelerometer")
        choice = input("Enter your choice (1 or 2): ")
        if choice == "1":
            calibrate_gyro()
        elif choice == "2":
            calibrate_accel()
        else:
            print("Invalid choice. Please enter 1 or 2.")

if __name__ == '__main__':
    main()