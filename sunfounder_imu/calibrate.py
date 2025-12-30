from sunfounder_imu.imu import IMU
import time
import threading

imu = IMU()

PROGRESS_BAR_COLOR_FG="\033[92m"
PROGRESS_BAR_COLOR_BG="\033[49m"
PROGRESS_BAR_COLOR_BG_BLOCKED="\033[43m"
PROGRESS_BAR_RESTORE_FG="\033[90m"
PROGRESS_BAR_RESTORE_BG="\033[49m"
PROGRESS_BAR_START_CHARACTER=""
PROGRESS_BAR_FILL_CHARACTER="━"
PROGRESS_BAR_EMPTY_CHARACTER="━"
PROGRESS_BAR_END_CHARACTER=""
COLOR_STYLE_RESET="\033[0m"

def progress_bar(current: int, total: int, msg: str="Progress: ", bar_length: int=50) -> None:
    """ Print a progress bar.
    
    Args:
        current (int): Current progress value
        total (int): Total progress value
        bar_length (int): Length of the progress bar (default: 20)
    """
    percent = float(current) / total
    fill_length = int(round(percent * bar_length))
    empty_length = bar_length - fill_length
    fill_color = f"{PROGRESS_BAR_COLOR_FG}{PROGRESS_BAR_COLOR_BG}"
    restore_color = f"{PROGRESS_BAR_RESTORE_FG}{PROGRESS_BAR_RESTORE_BG}"
    
    progress_bar = f"{PROGRESS_BAR_START_CHARACTER}{fill_color}{PROGRESS_BAR_FILL_CHARACTER * fill_length}{restore_color}{PROGRESS_BAR_EMPTY_CHARACTER * empty_length}{PROGRESS_BAR_END_CHARACTER}{COLOR_STYLE_RESET}"
    print(f"{msg}{progress_bar} {int(percent*100)}%", end="\r")

def calibrate_gyro():
    """ Calibrate the gyroscope.
    """
    TIMES = 500
    imu.calibrate_gyro_prepare()
    for i in range(TIMES):
        x, y, z = imu.calibrate_gyro_step()
        progress_bar(i, TIMES, msg=f"Calibrating gyroscope({x:8.2f}, {y:8.2f}, {z:8.2f})  ")
        time.sleep(0.01)
    gyro_offsets = imu.calibrate_gyro_finish()
    print("")
    return gyro_offsets

def calibrate_accel_mag():
    """ Calibrate the accelerometer.
    """

    key_pressed = False
    def key_press_thread():
        nonlocal key_pressed
        input("Press any key to stop calibration.")
        key_pressed = True
    thread = threading.Thread(target=key_press_thread)
    thread.start()

    imu.calibrate_accel_prepare()
    imu.calibrate_mag_prepare()
    while not key_pressed:
        accel_x, accel_y, accel_z = imu.calibrate_accel_step()
        mag_x, mag_y, mag_z = imu.calibrate_mag_step()
        print(f"\033[KAcceleration: ({accel_x:5.2f}, {accel_y:5.2f}, {accel_z:5.2f}), Magnetometer: ({mag_x:6.2f}, {mag_y:6.2f}, {mag_z:6.2f})", end="\r")
        time.sleep(0.01)
    accel_offsets, accel_scale, accel_max, accel_min = imu.calibrate_accel_finish()
    mag_offsets, mag_scales = imu.calibrate_mag_finish()

    return accel_offsets, accel_scale, accel_max, accel_min, mag_offsets, mag_scales

def main():
    print("\nCalibration\n")
    print("Calibrate gyroscope")
    print("Set the device down and DO NOT move it until the calibration is complete.")
    input("\nWhen you are ready, press ENTER to continue.")
    gyro_offsets = calibrate_gyro()
    print(f"Gyroscope offsets: {gyro_offsets}")
    print("Gyroscope calibration complete. data saved.")

    print("\n\nCalibrate accelerometer and magnetometer")
    print(f"SLOWLY spin the device in all axis and directions.")
    input("\nWhen you are ready, press ENTER to continue.")
    print("Calibrating accelerometer and magnetometer... press ENTER when done.")
    accel_offsets, accel_scale, accel_max, accel_min, mag_offsets, mag_scales = calibrate_accel_mag()
    print(f"Accelerometer offsets: {accel_offsets}")
    # print(f"Accelerometer scale: {accel_scale}")
    print(f"Accelerometer max: {accel_max}")
    print(f"Accelerometer min: {accel_min}")
    print(f"Magnetometer offsets: {mag_offsets}")
    print(f"Magnetometer scales: {mag_scales}")
    print("Accelerometer calibration complete. data saved.")

    print(f"\nCalibration complete. data saved to {imu.config_file}: ")
    with open(imu.config_file, 'r') as f:
        content = f.read()
        print(content)

if __name__ == '__main__':
    main()