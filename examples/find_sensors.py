# from sunfounder_imu.sensors import get_baro_sensor, get_accel_gyro_sensor, get_mag_sensor
# from sunfounder_imu._i2c import I2C

# addresses = I2C.scan()
# print(f"Found I2C addresses: {', '.join([f'0x{address:02x}' for address in addresses])}")
# accel_gyro = get_accel_gyro_sensor(addresses)
# mag = get_mag_sensor(addresses)
# baro = get_baro_sensor(addresses)
# if accel_gyro is not None:
#     print("Found accel_gyro sensor:")
#     print(f"  name: {accel_gyro.__class__.__name__}")
#     print(f"  address: 0x{accel_gyro.address:02x}")
# else:
#     print("No accel_gyro sensor found")
# if mag is not None:
#     print("Found mag sensor:")
#     print(f"  name: {mag.__class__.__name__}")
#     print(f"  address: 0x{mag.address:02x}")
# else:
#     print("No mag sensor found")
# if baro is not None:
#     print("Found baro sensor:")
#     print(f"  name: {baro.__class__.__name__}")
#     print(f"  address: 0x{baro.address:02x}")
# else:
#     print("No baro sensor found")

from sunfounder_imu import IMU

def main():
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

if __name__ == "__main__":
    main()