from sunfounder_imu.imu import IMU
import time
import numpy as np

from ._utils import format_3d_data

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

def main():
    print("\nCalibration\n")
    FACES = [
        "Z face up  ",
        "Z face down",
        "X face up  ",
        "X face down",
        "Y face up  ",
        "Y face down",
    ]

    TIMES = 50

    imu.calibrate_prepare()

    for face in FACES:
        print("\n\n=================")
        print(f"*  {face}  *")
        print("=================")
        print(f"Set the device down flat {face}, and DO NOT move it until this process is complete.")
        input("\nWhen you are ready, press ENTER to continue.")
        for i in range(TIMES):
            imu.calibrate_read()
            progress_bar(i+1, TIMES, msg="Reading data ")
            time.sleep(0.01)
        imu.calibrate_step()
    imu.calibrate_finish()

    print(f"\nCalibration complete. data saved to {imu.config_file}: ")
    with open(imu.config_file, 'r') as f:
        content = f.read()
        print(content)

if __name__ == '__main__':
    main()