
from .._base import _Base

class AccelGyroSensor(_Base):
    """
    Accelerometer and Gyroscope class to interface with accel_gyro sensor.

    Args:
        address (int): I2C address of the accel_gyro sensor.
    """
    
    G = 9.80665

    def __init__(self, address: int, *args, acc_offset: list=[0, 0, 0], acc_scale: list=[1.0, 1.0, 1.0], gyro_offset: list=[0, 0, 0], **kwargs):
        super().__init__(address, *args, **kwargs)
        self.address = address

        self.acc_offset = acc_offset
        self.acc_scale = acc_scale
        self.gyro_offset = gyro_offset

        self.temperature = None
        self.accel_x = None
        self.accel_y = None
        self.accel_z = None
        self.gyro_x = None
        self.gyro_y = None
        self.gyro_z = None
        self.gyro_cali_temp = None
        self.accel_cali_temp = None

    def read_temperature(self) -> float:
        raise NotImplementedError("read_temperature method not implemented")

    def read_accel(self) -> [float]:
        raise NotImplementedError("read_accel method not implemented")

    def read_gyro(self) -> [float]:
        raise NotImplementedError("read_gyro method not implemented")

    def read(self) -> tuple:
        ''' Read all data
        
        Returns:
            tuple: (accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z), temperature
        '''
        self.temperature = self.read_temperature()
        self.accel_x, self.accel_y, self.accel_z = self.read_accel()
        self.gyro_x, self.gyro_y, self.gyro_z = self.read_gyro()
        return (self.accel_x, self.accel_y, self.accel_z), (self.gyro_x, self.gyro_y, self.gyro_z), self.temperature

    def calibrate_gyro_prepare(self) -> None:
        ''' Prepare gyroscope calibration, clear temp datas
        '''
        self.gyro_cali_temp = []
    
    def calibrate_gyro_step(self) -> list:
        ''' Calibrate gyroscope
        
        Returns:
            list: Gyroscope data
        '''
        gyro_data = list(self.read_gyro(raw=True))
        self.gyro_cali_temp.append(gyro_data)
        return gyro_data
    
    def calibrate_gyro_finish(self) -> list:
        ''' Calibration finish, calculate offset

        Returns:
            list: Gyroscope offset
        '''
        if self.gyro_cali_temp is None:
            raise ValueError('Calibration data is empty')
        
        self.gyro_offset = [sum([v[i] for v in self.gyro_cali_temp]) / len(self.gyro_cali_temp) for i in range(3)]
        # Round to 2 decimal places
        self.gyro_offset = [round(v, 2) for v in self.gyro_offset]

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
        accel_data = list(self.read_accel(raw=True))
        self.accel_cali_temp.append(accel_data)
        return accel_data

    def calibrate_accel_finish(self) -> list:
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
        self.acc_scale = [accel_max[i] - accel_min[i] / 2 for i in range(3)]
        # Round to 2 decimal places
        self.acc_offset = [round(v, 2) for v in self.acc_offset]
        self.acc_scale = [round(v, 2) for v in self.acc_scale]

        return self.acc_offset, self.acc_scale, accel_max, accel_min

    def set_calibration_data(self, acc_offset: list=[0, 0, 0], acc_scale: list=[1.0, 1.0, 1.0], gyro_offset: list=[0, 0, 0]) -> None:
        ''' Set calibration data
        
        Args:
            acc_offset (list, optional): Acceleration offset. Defaults to [0, 0, 0].
            acc_scale (list, optional): Acceleration scale. Defaults to [1.0, 1.0, 1.0].
            gyro_offset (list, optional): Gyroscope offset. Defaults to [0, 0, 0].
        '''
        self.acc_offset = acc_offset
        self.acc_scale = acc_scale
        self.gyro_offset = gyro_offset

    def set_accel_offset(self, offset_list:list) -> None:
        ''' Set acceleration offset
        
        Args:
            offset_list (list): Acceleration offset.
        '''
        self.acc_offset = offset_list

    def set_accel_scale(self, scale_list:list) -> None:
        ''' Set acceleration scale
        
        Args:
            scale_list (list): Acceleration scale.
        '''
        self.acc_scale = scale_list

    def set_gyro_offset(self, offset_list:list) -> None:
        ''' Set gyroscope offset
        
        Args:
            offset_list (list): Gyroscope offset.
        '''
        self.gyro_offset = offset_list
