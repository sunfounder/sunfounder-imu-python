from sunfounder_imu.qmc6310 import QMC6310
import time 

mag = QMC6310()

def calibrate_magnetometer(seconds: int=20) -> None:
    print(f"Calibrate magnetometer. gentally move the device in all directions. after {seconds} seconds, the calibration will finish.")
    input("Enter to start calibration...")
    mag.calibrate_prepare()
    start_time = time.time()
    while True:
        mag.calibrate_step()
        time.sleep(0.001)
        during = time.time() - start_time
        print(f"Calibrating... {during:.2f}s")
        if during > seconds:
            break
    offsets, scales = mag.calibrate_finish()
    print("Calibration complete. data saved.")
    print(f"  Offsets: {offsets}")
    print(f"  Scales: {scales}")

calibrate_magnetometer()
