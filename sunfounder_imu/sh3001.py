#!/usr/bin/env python3
import time
from ._i2c import I2C
from .data_type import AccelDate, GyroDate

# Sensitivity
# 2g: 1G = 16384
# 4g: 1G = 8192
# 8g: 1G = 4096
# 16g:  1G = 2048


# region: General function
def bytes_toint(msb, lsb):
    '''
    Convert two bytes to signed integer (big endian)
    for little endian reverse msb, lsb arguments
    Can be used in an interrupt handler
    '''
    if not msb & 0x80:
        return msb << 8 | lsb  # +ve
    return -(((msb ^ 255) << 8) | (lsb ^ 255) + 1)


def default_wait():
    '''
    delay of 10 ms
    '''
    time.sleep(0.01)


def stop_func():
    return False


# endregion: General function

class SH3001(I2C):
    I2C_ADDRESSES = [0x36, 0x37]

    # region: Macro Definitions
    '''
    /******************************************************************
    *	SH3001 Registers Macro Definitions
    ******************************************************************/
    '''
    ACC_XL = 0x00
    ACC_XH = 0x01
    ACC_YL = 0x02
    ACC_YH = 0x03
    ACC_ZL = 0x04
    ACC_ZH = 0x05
    GYRO_XL = 0x06
    GYRO_XH = 0x07
    GYRO_YL = 0x08
    GYRO_YH = 0x09
    GYRO_ZL = 0x0A
    GYRO_ZH = 0x0B
    TEMP_DATAL = 0x0C
    CHIP_ID = 0x0F
    INT_STA0 = 0x10
    INT_STA1 = 0x11
    INT_STA2 = 0x12
    INT_STA3 = 0x14
    INT_STA4 = 0x15
    FIFO_STA0 = 0x16
    FIFO_STA1 = 0x17
    FIFO_DATA = 0x18
    TEMP_CONF0 = 0x20
    TEMP_CONF1 = 0x21

    ACC_CONF0 = 0x22  # accelerometer config 0x22-0x26
    ACC_CONF1 = 0x23
    ACC_CONF2 = 0x25
    ACC_CONF3 = 0x26

    GYRO_CONF0 = 0x28  # gyroscope config 0x28-0x2B
    GYRO_CONF1 = 0x29
    GYRO_CONF2 = 0x2B

    SPI_CONF = 0x32
    FIFO_CONF0 = 0x35
    FIFO_CONF1 = 0x36
    FIFO_CONF2 = 0x37
    FIFO_CONF3 = 0x38
    FIFO_CONF4 = 0x39
    MI2C_CONF0 = 0x3A
    MI2C_CONF1 = 0x3B
    MI2C_CMD0 = 0x3C
    MI2C_CMD1 = 0x3D
    MI2C_WR = 0x3E
    MI2C_RD = 0x3F
    INT_ENABLE0 = 0x40
    INT_ENABLE1 = 0x41
    INT_CONF = 0x44
    INT_LIMIT = 0x45
    ORIEN_INTCONF0 = 0x46
    ORIEN_INTCONF1 = 0x47
    ORIEN_INT_LOW = 0x48
    ORIEN_INT_HIGH = 0x49
    ORIEN_INT_SLOPE_LOW = 0x4A
    ORIEN_INT_SLOPE_HIGH = 0x4B
    ORIEN_INT_HYST_LOW = 0x4C
    ORIEN_INT_HYST_HIGH = 0x4D
    FLAT_INT_CONF = 0x4E
    ACT_INACT_INT_CONF = 0x4F
    ACT_INACT_INT_LINK = 0x50
    TAP_INT_THRESHOLD = 0x51
    TAP_INT_DURATION = 0x52
    TAP_INT_LATENCY = 0x53
    DTAP_INT_WINDOW = 0x54
    ACT_INT_THRESHOLD = 0x55
    ACT_INT_TIME = 0x56
    INACT_INT_THRESHOLDL = 0x57
    INACT_INT_TIME = 0x58
    HIGHLOW_G_INT_CONF = 0x59
    HIGHG_INT_THRESHOLD = 0x5A
    HIGHG_INT_TIME = 0x5B
    LOWG_INT_THRESHOLD = 0x5C
    LOWG_INT_TIME = 0x5D
    FREEFALL_INT_THRES = 0x5E
    FREEFALL_INT_TIME = 0x5F
    INT_PIN_MAP0 = 0x79
    INT_PIN_MAP1 = 0x7A
    INACT_INT_THRESHOLDM = 0x7B
    INACT_INT_THRESHOLDH = 0x7C
    INACT_INT_1G_REFL = 0x7D
    INACT_INT_1G_REFH = 0x7E
    SPI_REG_ACCESS = 0x7F
    GYRO_CONF3 = 0x8F
    GYRO_CONF4 = 0x9F
    GYRO_CONF5 = 0xAF
    AUX_I2C_CONF = 0xFD
    '''
    /******************************************************************
    *	ACC Config Macro Definitions
    ******************************************************************/
    '''
    ODR_1000HZ = 0x00
    ODR_500HZ = 0x01
    ODR_250HZ = 0x02
    ODR_125HZ = 0x03
    ODR_63HZ = 0x04
    ODR_31HZ = 0x05
    ODR_16HZ = 0x06
    ODR_2000HZ = 0x08
    ODR_4000HZ = 0x09
    ODR_8000HZ = 0x0A
    ODR_16000HZ = 0x0B
    ODR_32000HZ = 0x0C

    ACC_RANGE_16G = 0x02
    ACC_RANGE_8G = 0x03
    ACC_RANGE_4G = 0x04
    ACC_RANGE_2G = 0x05

    ACC_ODRX040 = 0x00
    ACC_ODRX025 = 0x20
    ACC_ODRX011 = 0x40
    ACC_ODRX004 = 0x60

    ACC_FILTER_EN = 0x00
    ACC_FILTER_DIS = 0x80
    '''
    /******************************************************************
    *	GYRO Config Macro Definitions
    ******************************************************************/
    '''
    GYRO_RANGE_125 = 0x02
    GYRO_RANGE_250 = 0x03
    GYRO_RANGE_500 = 0x04
    GYRO_RANGE_1000 = 0x05
    GYRO_RANGE_2000 = 0x06

    GYRO_ODRX00 = 0x00
    GYRO_ODRX01 = 0x04
    GYRO_ODRX02 = 0x08
    GYRO_ODRX03 = 0x0C

    GYRO_FILTER_EN = 0x00
    GYRO_FILTER_DIS = 0x10
    '''
    /******************************************************************
    *	Temperature Config Macro Definitions
    ******************************************************************/
    '''
    TEMP_ODR_500 = 0x00
    TEMP_ODR_250 = 0x10
    TEMP_ODR_125 = 0x20
    TEMP_ODR_63 = 0x30

    TEMP_EN = 0x80
    TEMP_DIS = 0x00
    '''
    /******************************************************************
    *	INT Config Macro Definitions
    ******************************************************************/
    '''
    INT_LOWG = 0x8000
    INT_HIGHG = 0x4000
    INT_INACT = 0x2000
    INT_ACT = 0x1000
    INT_DOUBLE_TAP = 0x0800
    INT_TAP = 0x0400
    INT_FLAT = 0x0200
    INT_ORIENTATION = 0x0100
    INT_FIFO_GYRO = 0x0010
    INT_GYRO_READY = 0x0008
    INT_ACC_FIFO = 0x0004
    INT_ACC_READY = 0x0002
    INT_FREE_FALL = 0x0001
    INT_UP_DOWN_Z = 0x0040

    INT_ENABLE = 0x01
    INT_DISABLE = 0x00

    INT_MAP_INT1 = 0x01
    INT_MAP_INT = 0x00

    INT_LEVEL_LOW = 0x80
    INT_LEVEL_HIGH = 0x7F
    INT_NO_LATCH = 0x40
    INT_LATCH = 0xBF
    INT_CLEAR_ANY = 0x10
    INT_CLEAR_STATUS = 0xEF
    INT_INT1_NORMAL = 0x04
    INT_INT1_OD = 0xFB
    INT_INT_NORMAL = 0x01
    INT_INT_OD = 0xFE
    '''
    /******************************************************************
    *	Orientation Blocking Config Macro Definitions
    ******************************************************************/
    '''
    ORIENT_BLOCK_MODE0 = 0x00
    ORIENT_BLOCK_MODE1 = 0x04
    ORIENT_BLOCK_MODE2 = 0x08
    ORIENT_BLOCK_MODE3 = 0x0C

    ORIENT_SYMM = 0x00
    ORIENT_HIGH_ASYMM = 0x01
    ORIENT_LOW_ASYMM = 0x02
    '''
    /******************************************************************
    *	Flat Time Config Macro Definitions
    ******************************************************************/
    '''
    FLAT_TIME_500MS = 0x40
    FLAT_TIME_1000MS = 0x80
    FLAT_TIME_2000MS = 0xC0
    '''
    /******************************************************************
    *	ACT and INACT Int Config Macro Definitions
    ******************************************************************/
    '''
    ACT_AC_MODE = 0x80
    ACT_DC_MODE = 0x00
    ACT_X_INT_EN = 0x40
    ACT_X_INT_DIS = 0x00
    ACT_Y_INT_EN = 0x20
    ACT_Y_INT_DIS = 0x00
    ACT_Z_INT_EN = 0x10
    ACT_Z_INT_DIS = 0x00

    INACT_AC_MODE = 0x08
    INACT_DC_MODE = 0x00
    INACT_X_INT_EN = 0x04
    INACT_X_INT_DIS = 0x00
    INACT_Y_INT_EN = 0x02
    INACT_Y_INT_DIS = 0x00
    INACT_Z_INT_EN = 0x01
    INACT_Z_INT_DIS = 0x00

    LINK_PRE_STA = 0x01
    LINK_PRE_STA_NO = 0x00
    '''
    /******************************************************************
    *	TAP Int Config Macro Definitions
    ******************************************************************/
    '''
    TAP_X_INT_EN = 0x08
    TAP_X_INT_DIS = 0x00
    TAP_Y_INT_EN = 0x04
    TAP_Y_INT_DIS = 0x00
    TAP_Z_INT_EN = 0x02
    TAP_Z_INT_DIS = 0x00
    '''
    /******************************************************************
    *	HIGHG Int Config Macro Definitions
    ******************************************************************/
    '''

    HIGHG_ALL_INT_EN = 0x80
    HIGHG_ALL_INT_DIS = 0x00
    HIGHG_X_INT_EN = 0x40
    HIGHG_X_INT_DIS = 0x00
    HIGHG_Y_INT_EN = 0x20
    HIGHG_Y_INT_DIS = 0x00
    HIGHG_Z_INT_EN = 0x10
    HIGHG_Z_INT_DIS = 0x00
    '''
    /******************************************************************
    *	LOWG Int Config Macro Definitions
    ******************************************************************/
    '''
    LOWG_ALL_INT_EN = 0x01
    LOWG_ALL_INT_DIS = 0x00
    '''
    /******************************************************************
    *	SPI Interface Config Macro Definitions
    ******************************************************************/
    '''
    SPI_3_WIRE = 0x01
    SPI_4_WIRE = 0x00
    '''
    /******************************************************************
    *	FIFO Config Macro Definitions
    ******************************************************************/
    '''
    FIFO_MODE_DIS = 0x00
    FIFO_MODE_FIFO = 0x01
    FIFO_MODE_STREAM = 0x02
    FIFO_MODE_TRIGGER = 0x03

    FIFO_ACC_DOWNS_EN = 0x80
    FIFO_ACC_DOWNS_DIS = 0x00
    FIFO_GYRO_DOWNS_EN = 0x08
    FIFO_GYRO_DOWNS_DIS = 0x00

    FIFO_FREQ_X1_2 = 0x00
    FIFO_FREQ_X1_4 = 0x01
    FIFO_FREQ_X1_8 = 0x02
    FIFO_FREQ_X1_16 = 0x03
    FIFO_FREQ_X1_32 = 0x04
    FIFO_FREQ_X1_64 = 0x05
    FIFO_FREQ_X1_128 = 0x06
    FIFO_FREQ_X1_256 = 0x07

    FIFO_EXT_Z_EN = 0x2000
    FIFO_EXT_Y_EN = 0x1000
    FIFO_EXT_X_EN = 0x0080
    FIFO_TEMPERATURE_EN = 0x0040
    FIFO_GYRO_Z_EN = 0x0020
    FIFO_GYRO_Y_EN = 0x0010
    FIFO_GYRO_X_EN = 0x0008
    FIFO_ACC_Z_EN = 0x0004
    FIFO_ACC_Y_EN = 0x0002
    FIFO_ACC_X_EN = 0x0001
    FIFO_ALL_DIS = 0x0000
    '''
    /******************************************************************
    *	AUX I2C Config Macro Definitions
    ******************************************************************/
    '''
    MI2C_NORMAL_MODE = 0x00
    MI2C_BYPASS_MODE = 0x01

    MI2C_READ_ODR_200HZ = 0x00
    MI2C_READ_ODR_100HZ = 0x10
    MI2C_READ_ODR_50HZ = 0x20
    MI2C_READ_ODR_25HZ = 0x30

    MI2C_FAIL = 0x20
    MI2C_SUCCESS = 0x10

    MI2C_READ_MODE_AUTO = 0x40
    MI2C_READ_MODE_MANUAL = 0x00
    '''
    /******************************************************************
    *	Other Macro Definitions
    ******************************************************************/
    '''
    TRUE = 0
    FALSE = 1

    NORMAL_MODE = 0x00
    SLEEP_MODE = 0x01
    POWERDOWN_MODE = 0x02

    # endregion: Macro Definitions

    # init
    def __init__(self, address=None):
        if address is None:
            addresses = I2C.scan(search=self.I2C_ADDRESSES)
            if addresses:
                address = addresses[0]
        super().__init__(address=address)
        if not self.is_avaliable():
            raise IOError("SH3001 is not avaliable")
        self.init()
        self.acc_offset = [0, 0, 0]
        self.acc_max = [0, 0, 0]
        self.acc_min = [0, 0, 0]

        self.gyro_offset = [0, 0, 0]
        self.data_vector = [0, 0, 0]

    def new_list(self, value):
        return [value for i in range(3)]

    def calibrate(self, aram, stopfunc=stop_func, waitfunc=default_wait):
        '''
        calibration routine, sets cal
        '''
        count = 0
        if aram == 'acc':
            while True:
                waitfunc()
                self.data_vector = self._read()[0]

                self.acc_max = list(map(max, self.acc_max, self.data_vector))
                self.acc_min = list(map(min, self.acc_min, self.data_vector))
                self.acc_offset = list(
                    map(lambda a, b: (a + b) / 2, self.acc_max, self.acc_min))
                print('\033[K\rmax_list: %s   min_list: %s' %
                      (self.acc_max, self.acc_min),
                      end="",
                      flush=True)
        elif aram == 'gyro':
            sum_list = [0, 0, 0]
            count = 0
            for i in range(503):
                if i > 2:
                    sum_list = [
                        sum_list[i] + self.sh3001_getimudata('gyro', 'xyz')[i]
                        for i in range(3)
                    ]
            self.gyro_offset = [
                round(sum_list[i], 2) / 500.0 for i in range(3)
            ]
            print("gyro_offset:", self.gyro_offset)

        else:
            raise ValueError('aram must be acc or gyro')

    def init(self):
        regData = [0]
        i = 0
        while ((regData[0] != 0x61) and (i < 3)):
            regData = self.mem_read(1, self.CHIP_ID)
            i += 1
            if (i == 3) and (regData[0] != 0x61):
                return False

        self.reset()
        self.set_acceleration_configuration(self.ODR_500HZ, self.ACC_RANGE_2G,
                               self.ACC_ODRX025,
                               self.ACC_FILTER_EN)
        self.set_gyroscope_configuration(self.ODR_500HZ,
                                self.GYRO_RANGE_2000,
                                self.GYRO_RANGE_2000,
                                self.GYRO_RANGE_2000,
                                self.GYRO_ODRX00,
                                self.GYRO_FILTER_EN)
        self.set_temperature_configuration(self.TEMP_ODR_63, self.TEMP_EN)

        return True

    def reset(self):
        # soft reset
        regData = 0x73
        self.mem_write(regData, self.address)
        time.sleep(0.05)

        # ADCreset
        regData = 0x02
        self.mem_write(regData, self.address)
        regData = 0xC1
        self.mem_write(regData, self.address)
        regData = 0xC2
        self.mem_write(regData, self.address)
        regData = 0x00
        self.mem_write(regData, self.address)

        # CVA reset
        regData = 0x18
        self.mem_write(regData, self.address)
        regData = 0x00
        self.mem_write(regData, self.address)
        time.sleep(0.01)

    def set_acceleration_configuration(self, accODR, accRange, accCutOffFreq,
                          accFilterEnble):
        # enable acc digital filter
        regData = self.mem_read(1, self.ACC_CONF0)
        regData[0] |= 0x01
        self.mem_write(regData, self.ACC_CONF0)

        # set acc ODR
        self.mem_write(accODR, self.ACC_CONF1)

        # set acc Range
        self.mem_write(accRange, self.ACC_CONF2)
        regData = self.mem_read(1, self.ACC_CONF2)
        # print(regData)

        # bypass acc low pass filter or not
        regData = self.mem_read(1, self.ACC_CONF3)
        regData[0] &= 0x17
        regData[0] |= (accCutOffFreq | accFilterEnble)
        self.mem_write(regData, self.ACC_CONF3)

    def set_gyroscope_configuration(self, gyroODR, gyroRangeX, gyroRangeY, gyroRangeZ,
                           gyroCutOffFreq, gyroFilterEnble):
        regData = self.mem_read(1, self.GYRO_CONF0)
        regData[0] |= 0x01
        self.mem_write(regData, self.GYRO_CONF0)

        # set gyro ODR
        self.mem_write(gyroODR, self.GYRO_CONF1)

        # set acc Range
        self.mem_write(gyroRangeX, self.GYRO_CONF3)
        self.mem_write(gyroRangeY, self.GYRO_CONF4)
        self.mem_write(gyroRangeZ, self.GYRO_CONF5)

        # bypass acc low pass filter or not
        regData = self.mem_read(1, self.GYRO_CONF2)
        regData[0] &= 0xE3
        regData[0] |= (gyroCutOffFreq | gyroFilterEnble)
        self.mem_write(regData, self.GYRO_CONF2)

    def set_temperature_configuration(self, tempODR, tempEnable):
        regData = self.mem_read(1, self.TEMP_CONF0)
        regData[0] &= 0x4F
        regData[0] |= (tempODR | tempEnable)
        self.mem_write(regData, self.TEMP_CONF0)
        regData = self.mem_read(1, self.TEMP_CONF0)

    # return accData,gyroData
    def _read(self):
        try:
            regData = self.mem_read(12, self.ACC_XL)

            accel_x = bytes_toint(regData[1], regData[0])
            accel_y = bytes_toint(regData[3], regData[2])
            accel_z = bytes_toint(regData[5], regData[4])

            gyro_x = bytes_toint(regData[7], regData[6])
            gyro_y = bytes_toint(regData[9], regData[8])
            gyro_z = bytes_toint(regData[11], regData[10])

            return AccelDate(accel_x, accel_y, accel_z), GyroDate(gyro_x, gyro_y, gyro_z)
        except Exception as e:
            return False

    def read_temperature(self):
        tempref = [0, 0]
        regData = self.mem_read(2, self.TEMP_CONF0)
        tempref[0] = regData[0] & 0x0F << 8 | regData[1]

        regData = self.mem_read(2, self.TEMP_DATAL)
        tempref[1] = regData[1] & 0x0F << 8 | regData[0]

        return (tempref[1] - tempref[0]) / 16.0 + 25.0

    def read(self):
        accData, gyroData = self._read()
        temperature = self.read_temperature()
        accData_List = [data - self.acc_offset[i] for i, data in enumerate(accData.list())]
        gyroData_List = [data - self.gyro_offset[i] for i, data in enumerate(gyroData.list())]
        accData = AccelDate(*accData_List)
        gyroData = GyroDate(*gyroData_List)

        return accData, gyroData, temperature

    def set_offset(self, offset_list=None):
        if offset_list == None:
            offset_list = self.acc_offset

    def acc_calibrate_cmd(self):
        try:
            print(
                'Calibration start!\nRotate the device for 720 degree in all 3 axis\nPress [Ctrl] + [C] if finish'
            )
            while True:
                self.calibrate('acc')
        except KeyboardInterrupt:
            print("")
            self.set_offset(self.acc_offset)
            print('offset: ', self.acc_offset)
