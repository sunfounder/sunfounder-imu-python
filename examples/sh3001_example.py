#!/usr/bin/env python3
import time
from sunfounder_imu._i2c import I2C
from sunfounder_imu.data_type import AccelDate, GyroDate
import struct
from typing import Optional

class SH3001():
    I2C_ADDRESSES = [0x36, 0x37]

    # region: Macro Definitions
    REG_ACC_X = 0x00
    ''' Register address for acceleration x-axis data '''
    REG_ACC_Y = 0x02
    ''' Register address for acceleration y-axis data '''
    REG_ACC_Z = 0x04
    ''' Register address for acceleration z-axis data '''
    REG_GYRO_X = 0x06
    ''' Register address for gyroscope x-axis data '''
    REG_GYRO_Y = 0x08
    ''' Register address for gyroscope y-axis data '''
    REG_GYRO_Z = 0x0A
    ''' Register address for gyroscope z-axis data '''
    REG_TEMP_DATA = 0x0C
    ''' Register address for temperature data '''
    REG_CHIP_ID = 0x0F
    ''' Register address for chip ID '''

    REG_TEMP_CONF0 = 0x20
    ''' Register address for temperature config 0 '''
    REG_TEMP_CONF1 = 0x21
    ''' Register address for temperature config 1 '''

    REG_ACC_CONF0 = 0x22
    ''' Register address for accelerometer config 0 '''
    REG_ACC_CONF1 = 0x23
    ''' Register address for accelerometer config 1 '''
    REG_ACC_CONF2 = 0x25
    ''' Register address for accelerometer config 2 '''
    REG_ACC_CONF3 = 0x26
    ''' Register address for accelerometer config 3 '''

    REG_GYRO_CONF0 = 0x28
    ''' Register address for gyroscope config 0 '''
    REG_GYRO_CONF1 = 0x29
    ''' Register address for gyroscope config 1 '''
    REG_GYRO_CONF2 = 0x2B
    ''' Register address for gyroscope config 2 '''
    REG_GYRO_CONF3 = 0x2C
    ''' Register address for gyroscope config 3 '''
    REG_GYRO_CONF4 = 0x2D
    ''' Register address for gyroscope config 4 '''
    REG_GYRO_CONF5 = 0x2E
    ''' Register address for gyroscope config 5 '''

    REG_SPI_CONF = 0x32
    ''' Register address for SPI config '''

    REG_FIFO_CONF0 = 0x35
    ''' Register address for FIFO config 0 '''
    REG_FIFO_CONF1 = 0x36
    ''' Register address for FIFO config 1 '''
    REG_FIFO_CONF2 = 0x37
    ''' Register address for FIFO config 2 '''
    REG_FIFO_CONF3 = 0x38
    ''' Register address for FIFO config 3 '''
    REG_FIFO_CONF4 = 0x39
    ''' Register address for FIFO config 4 '''

    REG_MI2C_CONF0 = 0x3A
    ''' Register address for Master I2C config 0 '''
    REG_MI2C_CONF1 = 0x3B
    ''' Register address for Master I2C config 1 '''
    REG_MI2C_CMD0 = 0x3C
    ''' Register address for Master I2C command 0 '''
    REG_MI2C_CMD1 = 0x3D
    ''' Register address for Master I2C command 1 '''
    REG_MI2C_WR = 0x3E
    ''' Register address for Master I2C write '''
    REG_MI2C_RD = 0x3F
    ''' Register address for Master I2C read '''

    '''
    /******************************************************************
    *	ACC Config Macro Definitions
    ******************************************************************/
    '''
    ACC_ODR_1000HZ = 0b0000
    ''' Accelerometer ODR 1000 Hz '''
    ACC_ODR_500HZ = 0b0001
    ''' Accelerometer ODR 500 Hz '''
    ACC_ODR_250HZ = 0b0010
    ''' Accelerometer ODR 250 Hz '''
    ACC_ODR_125HZ = 0b0011
    ''' Accelerometer ODR 125 Hz '''
    ACC_ODR_63HZ = 0b0100
    ''' Accelerometer ODR 63 Hz '''
    ACC_ODR_31HZ = 0b0101
    ''' Accelerometer ODR 31 Hz '''
    ACC_ODR_16HZ = 0b0110
    ''' Accelerometer ODR 16 Hz '''
    ACC_ODR_2000HZ = 0b1000
    ''' Accelerometer ODR 2000 Hz '''
    ACC_ODR_4000HZ = 0b1001
    ''' Accelerometer ODR 4000 Hz '''
    ACC_ODR_8000HZ = 0b1010
    ''' Accelerometer ODR 8000 Hz '''

    ACC_RANGE_16G = 0x02
    ''' Accelerometer range 16 g '''
    ACC_RANGE_8G = 0x03
    ''' Accelerometer range 8 g '''
    ACC_RANGE_4G = 0x04
    ''' Accelerometer range 4 g '''
    ACC_RANGE_2G = 0x05
    ''' Accelerometer range 2 g '''

    ACC_ODRX040 = 0x00
    ''' Accelerometer low pass filter cutout frequency ODR x 0.40 '''
    ACC_ODRX025 = 0x20
    ''' Accelerometer low pass filter cutout frequency ODR x 0.25 '''
    ACC_ODRX011 = 0x40
    ''' Accelerometer low pass filter cutout frequency ODR x 0.11 '''
    ACC_ODRX004 = 0x60
    ''' Accelerometer low pass filter cutout frequency ODR x 0.04 '''

    ACC_ODR_OPTIONS = [
        ACC_ODR_1000HZ,
        ACC_ODR_500HZ,
        ACC_ODR_250HZ,
        ACC_ODR_125HZ,
        ACC_ODR_63HZ,
        ACC_ODR_31HZ,
        ACC_ODR_16HZ,
        ACC_ODR_2000HZ,
        ACC_ODR_4000HZ,
        ACC_ODR_8000HZ,
    ]
    ''' Accelerometer ODR options '''

    ACC_RANGE_OPTIONS = [
        ACC_RANGE_2G,
        ACC_RANGE_4G,
        ACC_RANGE_8G,
        ACC_RANGE_16G,
    ]
    ''' Accelerometer range options '''

    ACC_LOW_PASS_FILTER_OPTIONS = [
        ACC_ODRX040,
        ACC_ODRX025,
        ACC_ODRX011,
        ACC_ODRX004,
    ]
    ''' Accelerometer low pass filter options '''


    GYRO_ODR_1000HZ = 0b0000
    ''' Gyroscope ODR 1000 Hz '''
    GYRO_ODR_500HZ = 0b0001
    ''' Gyroscope ODR 500 Hz '''
    GYRO_ODR_250HZ = 0b0010
    ''' Gyroscope ODR 250 Hz '''
    GYRO_ODR_125HZ = 0b0011
    ''' Gyroscope ODR 125 Hz '''
    GYRO_ODR_63HZ = 0b0100
    ''' Gyroscope ODR 63 Hz '''
    GYRO_ODR_31HZ = 0b0101
    ''' Gyroscope ODR 31 Hz '''
    GYRO_ODR_2000HZ = 0b1000
    ''' Gyroscope ODR 2000 Hz '''
    GYRO_ODR_4000HZ = 0b1001
    ''' Gyroscope ODR 4000 Hz '''
    GYRO_ODR_8000HZ = 0b1010
    ''' Gyroscope ODR 8000 Hz '''
    GYRO_ODR_16000HZ = 0b1011
    ''' Gyroscope ODR 16000 Hz '''
    GYRO_ODR_32000HZ = 0b1100
    ''' Gyroscope ODR 32000 Hz '''

    GYRO_RANGE_125 = 0x02
    ''' Gyroscope range 125 dps '''
    GYRO_RANGE_250 = 0x03
    ''' Gyroscope range 250 dps '''
    GYRO_RANGE_500 = 0x04
    ''' Gyroscope range 500 dps '''
    GYRO_RANGE_1000 = 0x05
    ''' Gyroscope range 1000 dps '''
    GYRO_RANGE_2000 = 0x06
    ''' Gyroscope range 2000 dps '''

    GYRO_LPF_00 = 0b00
    ''' Gyroscope low pass filter cutout frequency see datasheet about Gyroscope digital LPF cut-off frequency configuration '''
    GYRO_LPF_01 = 0b01
    ''' Gyroscope low pass filter cutout frequency see datasheet about Gyroscope digital LPF cut-off frequency configuration '''
    GYRO_LPF_10 = 0b10
    ''' Gyroscope low pass filter cutout frequency see datasheet about Gyroscope digital LPF cut-off frequency configuration '''
    GYRO_LPF_11 = 0b11
    ''' Gyroscope low pass filter cutout frequency see datasheet about Gyroscope digital LPF cut-off frequency configuration '''

    GYRO_ODR_OPTIONS = [
        GYRO_ODR_1000HZ,
        GYRO_ODR_500HZ,
        GYRO_ODR_250HZ,
        GYRO_ODR_125HZ,
        GYRO_ODR_63HZ,
        GYRO_ODR_31HZ,
        GYRO_ODR_2000HZ,
        GYRO_ODR_4000HZ,
        GYRO_ODR_8000HZ,
        GYRO_ODR_16000HZ,
        GYRO_ODR_32000HZ,
    ]
    ''' Gyroscope ODR options '''

    GYRO_RANGE_OPTIONS = [
        GYRO_RANGE_125,
        GYRO_RANGE_250,
        GYRO_RANGE_500,
        GYRO_RANGE_1000,
        GYRO_RANGE_2000,
    ]
    ''' Gyroscope range options '''

    GYRO_RANGE_MAP = {
        GYRO_RANGE_125: 125,
        GYRO_RANGE_250: 250,
        GYRO_RANGE_500: 500,
        GYRO_RANGE_1000: 1000,
        GYRO_RANGE_2000: 2000,
    }
    ''' Gyroscope range map '''

    GYRO_LOW_PASS_FILTER_OPTIONS = [
        GYRO_LPF_00,
        GYRO_LPF_01,
        GYRO_LPF_10,
        GYRO_LPF_11,
    ]
    ''' Gyroscope low pass filter options '''

    TEMP_ODR_500 = 0b00
    ''' Temperature ODR 500 Hz '''
    TEMP_ODR_250 = 0b01
    ''' Temperature ODR 250 Hz '''
    TEMP_ODR_125 = 0b10
    ''' Temperature ODR 125 Hz '''
    TEMP_ODR_63 = 0b11
    ''' Temperature ODR 63 Hz '''

    TEMP_ODR_OPTIONS = [
        TEMP_ODR_500,
        TEMP_ODR_250,
        TEMP_ODR_125,
        TEMP_ODR_63,
    ]
    ''' Temperature ODR options '''

    # VALUE
    CHIP_ID = 0x61
    ACC_WORKMODE_NORMAL = 0x01
    ACC_WORKMODE_LOW_POWER = 0x02

    ACC_RANGE_MAP = {
        ACC_RANGE_2G: 2,
        ACC_RANGE_4G: 4,
        ACC_RANGE_8G: 8,
        ACC_RANGE_16G: 16,
    }

    # init
    def __init__(self, address=None):
        if address is None:
            addresses = I2C.scan(search=self.I2C_ADDRESSES)
            if addresses:
                address = addresses[0]

        self.i2c = I2C(address=address)
        self.accel_range = 16
        self.gyro_range = [2000, 2000, 2000]
        self.init()

        self.acc_offset = [0, 0, 0]
        self.acc_max = [0, 0, 0]
        self.acc_min = [0, 0, 0]

        self.gyro_offset = [0, 0, 0]
        self.data_vector = [0, 0, 0]

    def init(self) -> bool:
        ''' Initialize the sensor
        
        Returns:
            bool: True if initialization is successful, False otherwise
        '''
        self.chip_id = self.i2c.read_byte_data(self.REG_CHIP_ID)
        if self.chip_id != self.CHIP_ID:
            raise ValueError(f'chip id not match, expected 0x{self.CHIP_ID:02X}, got 0x{self.chip_id:02X}')

        self.set_acceleration_configuration(
            low_power_enable = False, 
            adc_dither_enable = False,
            filter_enable = True,
            odr = self.ACC_ODR_500HZ,
            range = self.ACC_RANGE_16G,
            low_pass_filter_enable = True,
            low_pass_filter = self.ACC_ODRX025)
        self.set_gyroscope_configuration(
            inactive_detect_enable=False,
            filter_enable=True,
            odr=self.GYRO_ODR_500HZ,
            range_x=self.GYRO_RANGE_2000,
            range_y=self.GYRO_RANGE_2000,
            range_z=self.GYRO_RANGE_2000)
        self.set_temperature_configuration(enable=True, odr=self.TEMP_ODR_125)
        self.ROOM_TEMP = self.i2c.read_word_data(self.REG_TEMP_CONF0) & 0x0FFF
        print(f"room temp: {self.ROOM_TEMP}")

        return True

    def set_acceleration_configuration(self,
            low_power_enable:Optional[bool]=None,
            adc_dither_enable:Optional[bool]=None,
            filter_enable:Optional[bool]=None,
            odr:Optional[bytes]=None,
            range:Optional[bytes]=None,
            low_pass_filter_enable:Optional[bool]=None,
            low_pass_filter:Optional[bytes]=None) -> None:
        ''' Set acceleration configuration

        Args:
            low_power_enable (Optional[bool], optional): Low power mode. Defaults to None.
            adc_dither_enable (Optional[bool], optional): ADC dither mode. Defaults to None.
            filter_enable (Optional[bool], optional): Digital filter enable. Defaults to None.
            odr (Optional[bytes], optional): Acceleration ODR. Defaults to None, options are SH3001.ACC_ODR_1000HZ, SH3001.ACC_ODR_500HZ, SH3001.ACC_ODR_250HZ, SH3001.ACC_ODR_125HZ, SH3001.ACC_ODR_63HZ, SH3001.ACC_ODR_31HZ, SH3001.ACC_ODR_16HZ, SH3001.ACC_ODR_2000HZ, SH3001.ACC_ODR_4000HZ, SH3001.ACC_ODR_8000HZ.
            odr (Optional[bytes], optional): Acceleration ODR. Defaults to None, options are SH3001.ACC_ODR_1000HZ, SH3001.ACC_ODR_500HZ, 
            range (Optional[bytes], optional): Acceleration range. Defaults to None, options are SH3001.ACC_RANGE_2G, SH3001.ACC_RANGE_4G, SH3001.ACC_RANGE_8G, SH3001.ACC_RANGE_16G.
            low_pass_filter_enable (Optional[bool], optional): Acceleration low pass filter enable. Defaults to None.
            low_pass_filter (Optional[bytes], optional): Acceleration low pass filter cut-off frequency configuration. Defaults to None, options are ODRx0.4: SH3001.ACC_ODRX040, ODRx0.25: SH3001.ACC_ODRX025, ODRx0.11: SH3001.ACC_ODRX011, ODRx0.04: SH3001.ACC_ODRX004.
        '''
        acc_conf = self.i2c.read_i2c_block_data(self.REG_ACC_CONF0, 4)
        if low_power_enable is not None:
            acc_conf[0] |= int(low_power_enable) << 7
        if adc_dither_enable is not None:
            acc_conf[0] |= int(adc_dither_enable) << 6
        if filter_enable is not None:
            acc_conf[0] |= int(filter_enable) << 0
        if odr is not None:
            if odr not in self.ACC_ODR_OPTIONS:
                raise ValueError(f'odr not in {self.ACC_ODR_OPTIONS}')
            acc_conf[0] |= odr
        if range is not None and range in self.ACC_RANGE_OPTIONS:
            if range not in self.ACC_RANGE_OPTIONS:
                raise ValueError(f'range not in {self.ACC_RANGE_OPTIONS}')
            acc_conf[0] |= range
            self.accel_range = self.ACC_RANGE_MAP[range]
        
        if low_pass_filter_enable is not None:
            acc_conf[1] |= int(low_pass_filter_enable) << 3
        if low_pass_filter is not None:
            if low_pass_filter not in self.ACC_LOW_PASS_FILTER_OPTIONS:
                raise ValueError(f'low_pass_filter not in {self.ACC_LOW_PASS_FILTER_OPTIONS}')
            acc_conf[1] |= low_pass_filter
        
        self.i2c.write_i2c_block_data(self.REG_ACC_CONF0, acc_conf)

    def set_gyroscope_configuration(self, 
        inactive_detect_enable:Optional[bool]=None,
        filter_enable:Optional[bool]=None,
        odr:Optional[bytes]=None,
        low_pass_filter_enable:Optional[bool]=None,
        low_pass_filter:Optional[bytes]=None,
        range_x:Optional[bytes]=None,
        range_y:Optional[bytes]=None,
        range_z:Optional[bytes]=None) -> None:
        ''' Gyroscope configuration
        
        Args:
            inactive_detect_enable (Optional[bool], optional): Inactive detect enable. Defaults to None.
            filter_enable (Optional[bool], optional): Digital filter enable. Defaults to None.
            odr (Optional[bytes], optional): Gyroscope ODR. Defaults to None, options are SH3001.GYRO_ODR_500HZ, SH3001.GYRO_ODR_200HZ, SH3001.GYRO_ODR_100HZ, SH3001.GYRO_ODR_50HZ, SH3001.GYRO_ODR_25HZ.
            low_pass_filter_enable (Optional[bool], optional): Gyroscope low pass filter enable. Defaults to None.
            low_pass_filter (Optional[bytes], optional): Gyroscope low pass filter cut-off frequency configuration. Defaults to None, options are SH3001.GYRO_LPF_00, SH3001.GYRO_LPF_01, SH3001.GYRO_LPF_10, SH3001.GYRO_LPF_11.
            range_x (Optional[bytes], optional): Gyroscope range x. Defaults to None, options are SH3001.GYRO_RANGE_2000, SH3001.GYRO_RANGE_1000, SH3001.GYRO_RANGE_500, SH3001.GYRO_RANGE_250.
            range_y (Optional[bytes], optional): Gyroscope range y. Defaults to None, options are SH3001.GYRO_RANGE_2000, SH3001.GYRO_RANGE_1000, SH3001.GYRO_RANGE_500, SH3001.GYRO_RANGE_250.
            range_z (Optional[bytes], optional): Gyroscope range z. Defaults to None, options are SH3001.GYRO_RANGE_2000, SH3001.GYRO_RANGE_1000, SH3001.GYRO_RANGE_500, SH3001.GYRO_RANGE_250.
        '''
        gyro_conf = self.i2c.read_i2c_block_data(self.REG_GYRO_CONF0, 6)
        if inactive_detect_enable is not None:
            gyro_conf[0] |= int(inactive_detect_enable) << 4
        if filter_enable is not None:
            gyro_conf[0] |= int(filter_enable) << 0
        if odr is not None:
            if odr not in self.GYRO_ODR_OPTIONS:
                raise ValueError(f'odr not in {self.GYRO_ODR_OPTIONS}')
            gyro_conf[1] |= odr
        if low_pass_filter_enable is not None:
            gyro_conf[2] |= int(low_pass_filter_enable) << 4
        if low_pass_filter is not None:
            if low_pass_filter not in self.GYRO_LOW_PASS_FILTER_OPTIONS:
                raise ValueError(f'low_pass_filter not in {self.GYRO_LOW_PASS_FILTER_OPTIONS}')
            gyro_conf[2] |= low_pass_filter << 2
        if range_x is not None:
            if range_x not in self.GYRO_RANGE_OPTIONS:
                raise ValueError(f'range_x not in {self.GYRO_RANGE_OPTIONS}')
            gyro_conf[3] |= range_x
            self.gyro_range[0] = self.GYRO_RANGE_MAP[range_x]
        if range_y is not None:
            if range_y not in self.GYRO_RANGE_OPTIONS:
                raise ValueError(f'range_y not in {self.GYRO_RANGE_OPTIONS}')
            gyro_conf[4] |= range_y
            self.gyro_range[1] = self.GYRO_RANGE_MAP[range_y]
        if range_z is not None:
            if range_z not in self.GYRO_RANGE_OPTIONS:
                raise ValueError(f'range_z not in {self.GYRO_RANGE_OPTIONS}')
            gyro_conf[5] |= range_z
            self.gyro_range[2] = self.GYRO_RANGE_MAP[range_z]
        
        self.i2c.write_i2c_block_data(self.REG_GYRO_CONF0, gyro_conf)

    def set_temperature_configuration(self, enable:Optional[bool]=None, odr:Optional[bytes]=None) -> None:
        ''' Temperature configuration
        
        Args:
            enable (Optional[bool], optional): Temperature enable. Defaults to None.
            odr (Optional[bytes], optional): Temperature ODR. Defaults to None, options are SH3001.TEMP_ODR_50HZ, SH3001.TEMP_ODR_25HZ, SH3001.TEMP_ODR_10HZ, SH3001.TEMP_ODR_5HZ, SH3001.TEMP_ODR_2HZ.
        '''
        conf = self.i2c.read_byte_data(self.REG_TEMP_CONF0)
        print(f"original temp conf: 0b{conf:08b}")
        if enable is not None:
            print(f"enable: {enable}")
            conf &= 0b01111111
            conf |= int(enable) << 7
        if odr is not None:
            print(f"odr: {odr}")
            if odr not in self.TEMP_ODR_OPTIONS:
                raise ValueError(f'odr not in {self.TEMP_ODR_OPTIONS}')
            conf &= 0b11001111
            conf |= odr << 4

        print(f"temperature conf: 0b{conf:08b}")
        self.i2c.write_byte_data(self.REG_TEMP_CONF0, conf)

    def read_temperature(self, data:Optional[list]=None) -> float:
        ''' Read temperature
        
        Returns:
            float: Temperature in Celsius
        '''
        if data is None:
            data = self.i2c.read_i2c_block_data(self.REG_TEMP_DATA, 2)
        print(f"data: {data}")
        data = bytes(data)
        data = struct.unpack_from('!h', data, 0)[0]
        print(f"raw temp: {data}")
        # Both temperature readings and room temperature are 12-bit unsigned values
        data = data & 0x0FFF
        temperature = (data - self.ROOM_TEMP) / 16.0 + 25.0
        return temperature

    def read_accel(self, data:Optional[list]=None, raw:Optional[bool]=False) -> AccelDate:
        ''' Read acceleration
        
        Args:
            data (Optional[list], optional): Acceleration data. Defaults to None. provide data to skip reading
            raw (Optional[bool], optional): Raw data. Defaults to False.

        Returns:
            AccelDate: Acceleration data
        '''
        if data is None:
            data = self.i2c.read_i2c_block_data(self.REG_ACC_X, 6)

        data = bytes(data)
        values = struct.unpack_from('hhh', data, 0)
        if not raw:
            # Convert acceleration data to g
            values = [v / (2 * self.accel_range) + self.accel_range for v in values]
            # apply offset
            values = [v - self.acc_offset[i] for i, v in enumerate(values)]

        return AccelDate(*values)

    def read_gyro(self, data:Optional[list]=None, raw:Optional[bool]=False) -> GyroDate:
        ''' Read gyroscope
        
        Args:
            data (Optional[list], optional): Gyroscope data. Defaults to None. provide data to skip reading
            raw (Optional[bool], optional): Raw data. Defaults to False.

        Returns:
            GyroDate: Gyroscope data
        '''
        if data is None:
            data = self.i2c.read_i2c_block_data(self.REG_GYRO_X, 6)

        data = bytes(data)
        values = struct.unpack_from('hhh', data, 0)
        if not raw:
            # Convert gyroscope data to dps
            values = [v / (2 * self.gyro_range[i]) + self.gyro_range[i] for i, v in enumerate(values)]
            # apply offset
            values = [v - self.gyro_offset[i] for i, v in enumerate(values)]

        return GyroDate(*values)

    def read(self) -> tuple:
        ''' Read all data
        
        Returns:
            tuple: Acceleration data, Gyroscope data, Temperature
        '''
        data = self.i2c.read_i2c_block_data(self.REG_ACC_X, 14)
        accel_data = self.read_accel(data[:6])
        gyro_data = self.read_gyro(data[6:12])
        temperature = self.read_temperature(data[12:])
        return accel_data, gyro_data, temperature

    def set_accel_offset(self, offset_list:list) -> None:
        ''' Set acceleration offset
        
        Args:
            offset_list (list): Acceleration offset.
        '''
        self.acc_offset = offset_list

    def set_gyro_offset(self, offset_list:list) -> None:
        ''' Set gyroscope offset
        
        Args:
            offset_list (list): Gyroscope offset.
        '''
        self.gyro_offset = offset_list

    def calibrate_gyro(self, times: int = 100) -> list:
        ''' Calibrate gyroscope
        
        Args:
            times (int, optional): Calibration times. Defaults to 100.
        '''
        datas = []
        for _ in range(times):
            gyro_data = self.read_gyro(raw=True)
            datas.append(gyro_data.list())
            time.sleep(0.01)
        self.gyro_offset = [sum([v[i] for v in datas]) / len(datas) for i in range(3)]
        return self.gyro_offset

    def calibrate_accel_prepare(self) -> None:
        ''' Prepare accelerometer calibration, clear temp datas
        '''
        self.accel_cali_temp = []

    def calibrate_accel_step(self) -> None:
        ''' Calibration accelerometer step, read and store raw data
        '''
        if self.accel_cali_temp is None:
            self.calibrate_accel_prepare()
        accel_data = self.read_accel(raw=True)
        self.accel_cali_temp.append(accel_data.list())

    def calibratetion_finish(self) -> list:
        ''' Calibration finish, calculate offset

        Returns:
            list: Acceleration offset
        '''
        if self.accel_cali_temp is None:
            raise ValueError('Calibration data is empty')
        
        # Calculate accelrometer offset
        # Get max and min values
        accel_max = list(map(max, *self.accel_cali_temp))
        accel_min = list(map(min, *self.accel_cali_temp))
        # Calculate offset
        self.acc_offset = [(accel_max[i] + accel_min[i]) / 2 for i in range(3)]
        return self.acc_offset


import time

imu = SH3001()
temperature = imu.read_temperature()
print(f"Temperature: {temperature:.2f} °C")

# while True:
#     accData, gyroData, temperature = imu.read()
#     print(f"Acceleration: ")
#     print(f"  x: {accData.x:.2f}")
#     print(f"  y: {accData.y:.2f}")
#     print(f"  z: {accData.z:.2f}")
#     print(f"Gyroscope: ")
#     print(f"  x: {gyroData.x:.2f}")
#     print(f"  y: {gyroData.y:.2f}")
#     print(f"  z: {gyroData.z:.2f}")
#     print(f"Temperature: {temperature:.2f} °C")
#     time.sleep(1)
