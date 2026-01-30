import struct
from typing import Optional

from .accel_gyro_sensor import AccelGyroSensor

from .._i2c import I2C
from .._utils import mapping

import time

class QMI8658B(AccelGyroSensor):
    
    I2C_ADDRESSES = [0x6B, 0x6A]

    # General Purpose Registers
    REG_WHO_AM_I = 0x00
    REG_REVISION_ID = 0x01

    # Setup and Control Registers
    REG_CTRL1 = 0x02 # SPI Interface and Sensor Enable
    REG_CTRL2 = 0x03 # Accelerometer: Output DataRate, Full Scale, Self-Test
    REG_CTRL3 = 0x04 # Gyroscope: Output DataRate, Full Scale, Self-Test
    # REG_CTRL4 = 0x05  # Reserved
    REG_CTRL5 = 0x06 # Low pass filter setting
    # REG_CTRL6 = 0x07  # Reserved
    REG_CTRL7 = 0x08  # Enable Sensors
    REG_CTRL8 = 0x09  # Motion Detection Control
    REG_CTRL9 = 0x0A  # Host Commands

    # Host Controlled Calibration Registers
    REG_CAL1_L = 0x0B  # CAL1_L – lower 8 bits. CAL1_H – upper 8 bits.
    REG_CAL1_H = 0x0C
    REG_CAL2_L = 0x0D  # CAL2_L – lower 8 bits. CAL2_H – upper 8 bits.
    REG_CAL2_H = 0x0E
    REG_CAL3_L = 0x0F  # CAL3_L – lower 8 bits. CAL3_H – upper 8 bits.
    REG_CAL3_H = 0x10
    REG_CAL4_L = 0x11  # CAL4_L – lower 8 bits. CAL4_H – upper 8 bits.
    REG_CAL4_H = 0x12

    # FIFO Registers
    REG_FIFO_WTM_TH = 0x13  # FIFO watermark level, in ODRs
    REG_FIFO_CTRL = 0x14  # FIFO Setup
    REG_FIFO_SMPL_CNT = 0x15  # FIFO sample count LSBs
    REG_FIFO_STATUS = 0x16  # FIFO Status
    REG_FIFO_DATA = 0x17  # FIFO Data

    # Status Registers
    REG_STATUSINT = 0x2D  # Sensor Data Availability with the Locking mechanism, CmdDone (CTRL9 protocol bit)
    REG_STATUS0 = 0x2E  # Output Data Availability
    REG_STATUS1 = 0x2F  # Miscellaneous Status: Any Motion, No Motion, Significant Motion, Tap

    # Timestamp Register
    REG_TIMESTAMP_LOW = 0x30  # Sample Time Stamp - lower 8 bits
    REG_TIMESTAMP_MID = 0x31  # Sample Time Stamp - middle 8 bits
    REG_TIMESTAMP_HIGH = 0x32  # Sample Time Stamp - upper 8 bits

    # Data Output Registers (16 bits 2's Complement Except COD Sensor Data)
    # Temperature Registers
    REG_TEMP_L = 0x33  # Temperature Output Data - lower 8 bits
    REG_TEMP_H = 0x34  # Temperature Output Data - upper 8 bits

    # Accelerometer Registers
    REG_AX_L = 0x35  # X-axis Acceleration - lower 8 bits
    REG_AX_H = 0x36  # X-axis Acceleration - upper 8 bits
    REG_AY_L = 0x37  # Y-axis Acceleration - lower 8 bits
    REG_AY_H = 0x38  # Y-axis Acceleration - upper 8 bits
    REG_AZ_L = 0x39  # Z-axis Acceleration - lower 8 bits
    REG_AZ_H = 0x3A  # Z-axis Acceleration - upper 8 bits

    # Gyroscope Registers
    REG_GX_L = 0x3B  # X-axis Angular Rate - lower 8 bits
    REG_GX_H = 0x3C  # X-axis Angular Rate - upper 8 bits
    REG_GY_L = 0x3D  # Y-axis Angular Rate - lower 8 bits
    REG_GY_H = 0x3E  # Y-axis Angular Rate - upper 8 bits
    REG_GZ_L = 0x3F  # Z-axis Angular Rate - lower 8 bits
    REG_GZ_H = 0x40  # Z-axis Angular Rate - upper 8 bits

    # COD Indication and General Purpose Registers
    REG_COD_STATUS = 0x46  # Calibration-On-Demand status register
    REG_dQW_L = 0x49  # General purpose register
    REG_dQW_H = 0x4A  # General purpose register
    REG_dQX_L = 0x4B  # General purpose register
    REG_dQX_H = 0x4C  # Reserved
    REG_dQY_L = 0x4D  # General purpose register
    REG_dQY_H = 0x4E  # Reserved
    REG_dQZ_L = 0x4F  # Reserved
    REG_dQZ_H = 0x50  # Reserved
    REG_dVX_L = 0x51  # General purpose register
    REG_dVX_H = 0x52  # General purpose register
    REG_dVY_L = 0x53  # General purpose register
    REG_dVY_H = 0x54  # General purpose register
    REG_dVZ_L = 0x55  # General purpose register
    REG_dVZ_H = 0x56  # General purpose register

    # Activity Detection Output Registers
    REG_TAP_STATUS = 0x59  # Axis, direction, number of detected Tap

    # Reset Register
    REG_RESET = 0x60  # Soft Reset Register

    # 

    # Data preset
    # =======================================================================
    # General
    WHO_AM_I = 0x05 # Device identifier 0x05 = to identifu the device is a QST sensor
    REVISION_ID = 0x7C # Device Revision ID

    # Accelerometer Output Data Rate (ODR) options
    # Normal
    ACC_ODR_8000HZ  = 0b0000
    ACC_ODR_4000HZ  = 0b0001
    ACC_ODR_2000HZ  = 0b0010
    ACC_ODR_1000HZ  = 0b0011
    ACC_ODR_500HZ   = 0b0100
    ACC_ODR_250HZ   = 0b0101
    ACC_ODR_125HZ   = 0b0110
    ACC_ODR_62_5HZ  = 0b0111
    ACC_ODR_31_25HZ = 0b1000
    # Accelerometer Full Scale options
    ACCEL_RANGE_2G = 0b00   # ±2g
    ACCEL_RANGE_4G = 0b01   # ±4g
    ACCEL_RANGE_8G = 0b10   # ±8g
    ACCEL_RANGE_16G = 0b11  # ±16g

    # Low Power
    ACC_ODR_128HZ   = 0b1100
    ACC_ODR_21HZ    = 0b1101
    ACC_ODR_11HZ    = 0b1110
    ACC_ODR_3HZ     = 0b1111

    # Gyroscope Output Data Rate (ODR) options
    GYRO_ODR_7174_4HZ = 0b0000  # 7174.4 Hz, Normal, 100%
    GYRO_ODR_3587_2HZ = 0b0001  # 3587.2 Hz, Normal, 100%
    GYRO_ODR_1793_6HZ = 0b0010  # 1793.6 Hz, Normal, 100%
    GYRO_ODR_896_8HZ = 0b0011   # 896.8 Hz, Normal, 100%
    GYRO_ODR_448_4HZ = 0b0100   # 448.4 Hz, Normal, 100%
    GYRO_ODR_224_2HZ = 0b0101   # 224.2 Hz, Normal, 100%
    GYRO_ODR_112_1HZ = 0b0110   # 112.1 Hz, Normal, 100%
    GYRO_ODR_56_05HZ = 0b0111   # 56.05 Hz, Normal, 100%
    GYRO_ODR_28_025HZ = 0b1000  # 28.025 Hz, Normal, 100%
    # ODR values 0x09 to 0x0F are not available

    # Gyroscope Full Scale options
    GYRO_RANGE_16DPS = 0b000   # ±16 dps
    GYRO_RANGE_32DPS = 0b001   # ±32 dps
    GYRO_RANGE_64DPS = 0b010   # ±64 dps
    GYRO_RANGE_128DPS = 0b011  # ±128 dps
    GYRO_RANGE_256DPS = 0b100  # ±256 dps
    GYRO_RANGE_512DPS = 0b101  # ±512 dps
    GYRO_RANGE_1024DPS = 0b110 # ±1024 dps
    GYRO_RANGE_2048DPS = 0b111 # ±2048 dps

    # Low-Pass Filter Mode options
    LPF_MODE_2_66 = 0b00   # 2.66% of ODR
    LPF_MODE_3_63 = 0b01   # 3.63% of ODR
    LPF_MODE_5_39 = 0b10   # 5.39% of ODR
    LPF_MODE_13_37 = 0b11  # 13.37% of ODR

    # SOFTWARE_RESET
    SOFTWARE_RESET = 0xB0
    SOFTWARE_RESET_OK = 0x80

    # MODES
    MODE_POWER_ON_DEFAULT = 0x00
    MODE_LOW_POWER = 0x01
    MODE_POWER_DOWN = 0x02
    MODE_NORMAL_ACCEL_ONLY = 0x03
    MODE_LOW_POWER_ACCEL_ONLY = 0x04
    MODE_SNOOZE_GYRO = 0x05
    MODE_GYRO_ONLY = 0x06
    MODE_ACCEL_GYRO = 0x07
    MODE_ACCEL_SNOOZE_GYRO = 0x08
    MODE_SOFTWARE_RESET = 0x09
    MODE_NO_POWER = 0x0A

    MODES = [
        MODE_POWER_ON_DEFAULT,
        MODE_LOW_POWER,
        MODE_POWER_DOWN,
        MODE_NORMAL_ACCEL_ONLY,
        MODE_LOW_POWER_ACCEL_ONLY,
        MODE_SNOOZE_GYRO,
        MODE_GYRO_ONLY,
        MODE_ACCEL_GYRO,
        MODE_ACCEL_SNOOZE_GYRO,
        MODE_SOFTWARE_RESET,
        MODE_NO_POWER,
    ]

    ACCEL_RANGES = {
        ACCEL_RANGE_2G: 2,
        ACCEL_RANGE_4G: 4,
        ACCEL_RANGE_8G: 8,
        ACCEL_RANGE_16G: 16,
    }

    GYRO_RANGES = {
        GYRO_RANGE_16DPS: 16,
        GYRO_RANGE_32DPS: 32,
        GYRO_RANGE_64DPS: 64,
        GYRO_RANGE_128DPS: 128,
        GYRO_RANGE_256DPS: 256,
        GYRO_RANGE_512DPS: 512,
        GYRO_RANGE_1024DPS: 1024,
        GYRO_RANGE_2048DPS: 2048,
    }

    def __init__(self, *args, address=None, **kwargs):
        if address is None:
            addresses = I2C.scan(search=self.I2C_ADDRESSES)
            if addresses:
                address = addresses[0]
            else:
                raise ValueError("QMI8658B not found")

        super().__init__(address, *args, **kwargs)

        self.accel_range = self.ACCEL_RANGES[self.ACCEL_RANGE_2G]
        self.gyro_range = self.GYRO_RANGES[self.GYRO_RANGE_16DPS]

        self.i2c = I2C(address=address)
        self.set_mode(self.MODE_ACCEL_GYRO)
        # Enable address auto-increment for burst read/write
        self.set_ctrl1(addr_ai=True)

    def set_mode(self, mode: int):
        ''' Set mode
        
        Args:
            mode (int): Mode to set
        '''
        self.reset()
        if mode not in self.MODES:
            raise ValueError(f"Invalid mode {mode}")
        if mode == self.MODE_LOW_POWER:
            self.set_ctrl1(sensor_disable=False)
            self.set_ctrl7(accel_en=False, gyro_en=False)
        elif mode == self.MODE_POWER_DOWN:
            self.set_ctrl1(sensor_disable=True)
            self.set_ctrl7(accel_en=False, gyro_en=False)
        elif mode == self.MODE_NORMAL_ACCEL_ONLY:
            self.set_ctrl7(accel_en=True, gyro_en=False)
        elif mode == self.MODE_LOW_POWER_ACCEL_ONLY:
            self.set_ctrl7(accel_en=True, gyro_en=False)
        elif mode == self.MODE_SNOOZE_GYRO:
            self.set_ctrl7(accel_en=False, gyro_en=True, gyro_snooze=True)
        elif mode == self.MODE_GYRO_ONLY:
            self.set_ctrl7(accel_en=False, gyro_en=True, gyro_snooze=False)
        elif mode == self.MODE_ACCEL_GYRO:
            self.set_ctrl7(accel_en=True, gyro_en=True, gyro_snooze=False)
        elif mode == self.MODE_ACCEL_SNOOZE_GYRO:
            self.set_ctrl7(accel_en=True, gyro_en=True, gyro_snooze=True)

    def reset(self):
        ''' Reset the device
        '''
        # Reset the device
        self.i2c.write_byte_data(self.REG_CTRL1, self.SOFTWARE_RESET)
        # Wait for maximum 15ms for the device to be ready
        time.sleep(0.015)
        # Check if the device is ready
        status = self.i2c.read_byte_data(self.REG_dQY_L)
        if status != self.SOFTWARE_RESET_OK:
            raise ValueError("QMI8658B reset failed")

    def set_ctrl1(self,
            sim: Optional[bool]=None, # Enables SPI interface with: 0 = 4-wire, 1 = 3-wire, default 0
            addr_ai: Optional[bool]=None, # Serial interface address: 0 = non-increment, 1 = auto incremental, default 0
            be: Optional[bool]=None, # Endianness: 0 = little endian, 1 = big endian, default 1
            int2_en: Optional[bool]=None, # INT2: 0 = high-z mode, 1 = output enabled, default 0
            int1_en: Optional[bool]=None, # INT1: 0 = high-z mode, 1 = output enabled, default 0
            fifo_int_sel: Optional[bool]=None, # FIFO interrupt selection: 0 = INT2, 1 = INT1, default 0
            sensor_disable: Optional[bool]=None, # Internal high-speed oscillator: 0 = enabled, 1 = disabled, default 0
        ):
        ctrl1 = self.i2c.read_byte_data(self.REG_CTRL1)
        if sim is not None:
            ctrl1 = (ctrl1 & 0b01111111) | ((0x01 if sim else 0x00) << 7)
        if addr_ai is not None:
            ctrl1 = (ctrl1 & 0b10111111) | ((0x01 if addr_ai else 0x00) << 6)
        if be is not None:
            ctrl1 = (ctrl1 & 0b11011111) | ((0x01 if be else 0x00) << 5)
        if int2_en is not None:
            ctrl1 = (ctrl1 & 0b11101111) | ((0x01 if int2_en else 0x00) << 4)
        if int1_en is not None:
            ctrl1 = (ctrl1 & 0b11110111) | ((0x01 if int1_en else 0x00) << 3)
        if fifo_int_sel is not None:
            ctrl1 = (ctrl1 & 0b11111011) | ((0x01 if fifo_int_sel else 0x00) << 2)
        if sensor_disable is not None:
            ctrl1 = (ctrl1 & 0b11111110) | ((0x01 if sensor_disable else 0x00) << 0)
        self.i2c.write_byte_data(self.REG_CTRL1, ctrl1)

    def set_ctrl2(self,
            acc_odr: Optional[int]=None, # Accelerometer Output DataRate, default 0 (8000Hz)
            acc_fs: Optional[int]=None, # Accelerometer Full Scale, default 0 (±2g)
            acc_self_test: Optional[bool]=None, # Accelerometer Self-Test, default False (disabled)
        ):
        ctrl2 = self.i2c.read_byte_data(self.REG_CTRL2)
        if acc_odr is not None:
            # Assuming ODR is in bits 0-3
            ctrl2 = (ctrl2 & 0b11110000) | ((acc_odr & 0b00001111) << 0)
        if acc_fs is not None:
            # Assuming Full Scale is in bits 4-6
            ctrl2 = (ctrl2 & 0b10001111) | ((acc_fs & 0b00000111) << 4)
            self.accel_range = ACCEL_RANGE_MAP[acc_fs]
        if acc_self_test is not None:
            # Assuming Self-Test is in bits 0-1
            ctrl2 = (ctrl2 & 0b01111111) | ((0x01 if acc_self_test else 0x00) << 7)
        self.i2c.write_byte_data(self.REG_CTRL2, ctrl2)

    def set_ctrl3(self,
            gyro_self_test: Optional[bool]=None, # Gyro self-Test, default False (disabled)
            gyro_fs: Optional[int]=None, # Gyroscope Full Scale, default 0 (±16 dps)
            gyro_odr: Optional[int]=None, # Gyroscope Output DataRate, default 0 (7174.4 Hz)
        ):
        ctrl3 = self.i2c.read_byte_data(REG_CTRL3)
        if gyro_self_test is not None:
            # Gyro self-Test is in bit 7
            ctrl3 = (ctrl3 & 0b01111111) | ((0x01 if gyro_self_test else 0x00) << 7)
        if gyro_fs is not None:
            # Gyroscope Full Scale is in bits 4-6
            ctrl3 = (ctrl3 & 0b10001111) | ((gyro_fs & 0b00000111) << 4)
            self.gyro_range = GYRO_RANGE_MAP[gyro_fs]
        if gyro_odr is not None:
            # Gyroscope Output DataRate is in bits 0-3
            ctrl3 = (ctrl3 & 0b11110000) | ((gyro_odr & 0b00001111) << 0)
        self.i2c.write_byte_data(REG_CTRL3, ctrl3)

    def set_ctrl5(self,
            gyro_lpf_en: Optional[bool]=None, # Gyroscope Low-Pass Filter enable, default False (disabled)
            gyro_lpf_mode: Optional[int]=None, # Gyroscope Low-Pass Filter mode, default 0 (2.66% of ODR)
            accel_lpf_en: Optional[bool]=None, # Accelerometer Low-Pass Filter enable, default False (disabled)
            accel_lpf_mode: Optional[int]=None, # Accelerometer Low-Pass Filter mode, default 0 (2.66% of ODR)
        ):
        ctrl5 = self.i2c.read_byte_data(self.REG_CTRL5)
        if gyro_lpf_en is not None:
            # Gyroscope Low-Pass Filter enable is in bit 4
            ctrl5 = (ctrl5 & 0b11101111) | ((0x01 if gyro_lpf_en else 0x00) << 4)
        if gyro_lpf_mode is not None:
            # Gyroscope Low-Pass Filter mode is in bits 5-6
            ctrl5 = (ctrl5 & 0b10011111) | ((gyro_lpf_mode & 0b00000011) << 5)
        if accel_lpf_en is not None:
            # Accelerometer Low-Pass Filter enable is in bit 0
            ctrl5 = (ctrl5 & 0b11111110) | ((0x01 if accel_lpf_en else 0x00) << 0)
        if accel_lpf_mode is not None:
            # Accelerometer Low-Pass Filter mode is in bits 1-2
            ctrl5 = (ctrl5 & 0b11111001) | ((accel_lpf_mode & 0b00000011) << 1)
        self.i2c.write_byte_data(self.REG_CTRL5, ctrl5)

    def set_ctrl7(self,
            sync_sample: Optional[bool]=None, # SyncSample mode enable, default False (disabled)
            drdy_dis: Optional[bool]=None, # DRDY (Data Ready) disable, default False (enabled)
            gyro_snooze: Optional[bool]=None, # Gyroscope Snooze Mode, default False (Full Mode)
            gyro_en: Optional[bool]=None, # Gyroscope enable, default False (disabled)
            accel_en: Optional[bool]=None, # Accelerometer enable, default False (disabled)
        ):
        ctrl7 = self.i2c.read_byte_data(self.REG_CTRL7)
        if sync_sample is not None:
            # SyncSample mode is in bit 7
            ctrl7 = (ctrl7 & 0b01111111) | ((0x01 if sync_sample else 0x00) << 7)
        if drdy_dis is not None:
            # DRDY disable is in bit 5
            ctrl7 = (ctrl7 & 0b11011111) | ((0x01 if drdy_dis else 0x00) << 5)
        if gyro_snooze is not None:
            # Gyroscope Snooze Mode is in bit 4
            ctrl7 = (ctrl7 & 0b11101111) | ((0x01 if gyro_snooze else 0x00) << 4)
        if gyro_en is not None:
            # Gyroscope enable is in bit 1
            ctrl7 = (ctrl7 & 0b11111101) | ((0x01 if gyro_en else 0x00) << 1)
        if accel_en is not None:
            # Accelerometer enable is in bit 0
            ctrl7 = (ctrl7 & 0b11111110) | ((0x01 if accel_en else 0x00) << 0)
        self.i2c.write_byte_data(self.REG_CTRL7, ctrl7)

    def set_ctrl8(self,
            ctrl9_handshake_type: Optional[bool]=None, # CTRL9 handshake type, default False (use INT1)
            activity_int_sel: Optional[bool]=None, # Activity Detection interrupt selection, default False (INT2)
            sig_motion_en: Optional[bool]=None, # Significant Motion engine enable, default False (disabled)
            no_motion_en: Optional[bool]=None, # No Motion engine enable, default False (disabled)
            any_motion_en: Optional[bool]=None, # Any Motion engine enable, default False (disabled)
            tap_en: Optional[bool]=None, # Tap engine enable, default False (disabled)
        ):
        ctrl8 = self.i2c.read_byte_data(self.REG_CTRL8)
        if ctrl9_handshake_type is not None:
            # CTRL9 handshake type is in bit 7
            ctrl8 = (ctrl8 & 0b01111111) | ((0x01 if ctrl9_handshake_type else 0x00) << 7)
        if activity_int_sel is not None:
            # Activity Detection interrupt selection is in bit 6
            ctrl8 = (ctrl8 & 0b10111111) | ((0x01 if activity_int_sel else 0x00) << 6)
        if sig_motion_en is not None:
            # Significant Motion engine enable is in bit 3
            ctrl8 = (ctrl8 & 0b11110111) | ((0x01 if sig_motion_en else 0x00) << 3)
        if no_motion_en is not None:
            # No Motion engine enable is in bit 2
            ctrl8 = (ctrl8 & 0b11111011) | ((0x01 if no_motion_en else 0x00) << 2)
        if any_motion_en is not None:
            # Any Motion engine enable is in bit 1
            ctrl8 = (ctrl8 & 0b11111101) | ((0x01 if any_motion_en else 0x00) << 1)
        if tap_en is not None:
            # Tap engine enable is in bit 0
            ctrl8 = (ctrl8 & 0b11111110) | ((0x01 if tap_en else 0x00) << 0)
        self.i2c.write_byte_data(self.REG_CTRL8, ctrl8)

    def set_accel_range(self, range:int):
        ''' Set acceleration range
        
        Args:
            range (int): Acceleration range
        '''
        if range not in self.ACCEL_RANGES:
            raise ValueError(f'Invalid acceleration range {range}')

        self.set_ctrl2(acc_fs=range)

    def set_gyro_range(self, range:int):
        ''' Set gyroscope range
        
        Args:
            range (int): Gyroscope range
        '''
        if range not in self.GYRO_RANGES:
            raise ValueError(f'Invalid gyroscope range {range}')

        self.set_ctrl3(gyro_fs=range)

    def read_raw_temperature(self, data:Optional[list]=None) -> float:
        ''' Read raw temperature
        
        Returns:
            float: Temperature in Celsius
        '''
        if data is None:
            data = self.i2c.read_i2c_block_data(self.REG_TEMP_L, 2)
        data = bytes(data)
        data = struct.unpack_from('!h', data, 0)[0]
        temperature = data / 256.0

        return temperature

    def read_raw_accel(self, data:Optional[list]=None) -> tuple:
        ''' Read raw acceleration
        
        Args:
            data (Optional[list], optional): Acceleration data. Defaults to None. provide data to skip reading

        Returns:
            tuple: Acceleration data
        '''
        if data is None:
            data = self.i2c.read_i2c_block_data(self.REG_AX_L, 6)

        data = bytes(data)
        values = struct.unpack_from('hhh', data, 0)
        # Convert acceleration data to g
        values = [mapping(v, -0x8000, 0x7FFF, -self.accel_range, self.accel_range) for v in values]

        return tuple(values)

    def read_raw_gyro(self, data:Optional[list]=None) -> tuple:
        ''' Read raw gyroscope
        
        Args:
            data (Optional[list], optional): Gyroscope data. Defaults to None. provide data to skip reading

        Returns:
            tuple: Gyroscope data
        '''
        if data is None:
            data = self.i2c.read_i2c_block_data(self.REG_GX_L, 6)

        data = bytes(data)
        values = struct.unpack_from('hhh', data, 0)
        # Convert gyroscope data to dps
        values = [mapping(v, -0x8000, 0x7FFF, -self.gyro_range, self.gyro_range) for v in values]

        return tuple(values)

    def read_raw(self) -> tuple:
        ''' Read all raw data
        
        Returns:
            tuple: Acceleration data, Gyroscope data, Temperature
        '''
        data = self.i2c.read_i2c_block_data(self.REG_TEMP_L, 14)
        temperature = self.read_raw_temperature(data[:2])
        accel_data = self.read_raw_accel(data[2:8])
        gyro_data = self.read_raw_gyro(data[8:14])
        return accel_data, gyro_data, temperature
