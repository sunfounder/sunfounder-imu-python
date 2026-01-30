""" I2C bus read/write functions

A simple wrap for smbus2 library.

Example:
    import I2C

    >>> from fusion_hat.i2c import I2C

    Init I2C bus with address 0x17

    >>> i2c = I2C(address=0x17)

    Write a byte 0x00

    >>> i2c.write_byte(0x00)

    Write a byte 0x01 to register 0x01

    >>> i2c.write_byte_data(0x01, 0x01)

    Write a word 0x0001 to register 0x01

    >>> i2c.write_word_data(0x01, 0x0001)

    Write a word 0x0001 to register 0x01 in LSB first

    >>> i2c.write_word_data(0x01, 0x0001, lsb=True)

    Write a block of bytes to register 0x01

    >>> i2c.write_block_data(0x01, [0x00, 0x01, 0x02])

    Read a byte

    >>> print(i2c.read_byte())
    0

    Read a byte from register 0x01

    >>> print(i2c.read_byte_data(0x01))
    1

    Read a word from register 0x01

    >>> print(i2c.read_word_data(0x01))
    256

    Read a word from register 0x01 in LSB first

    >>> print(i2c.read_word_data(0x01, lsb=True))
    1

    Read a block of bytes from register 0x01

    >>> print(i2c.read_block_data(0x01))
    [0, 1, 2]
"""

#!/usr/bin/env python3
from smbus2 import SMBus, i2c_msg

from ._utils import retry
from ._base import _Base

class I2C(_Base):
    """ I2C bus read/write functions
    
    Args:
        address (int): I2C device address
        bus (int): I2C bus number
        log_level (int, str, optional): Log level, default is logging.INFO
    """

    RETRY = 5
    DEFAULT_BUS = 1

    def __init__(self, *args, address: int = None, bus: int = DEFAULT_BUS, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._bus = bus
        self._smbus = SMBus(self._bus)

        if isinstance(address, list):
            connected_devices = self.scan(all=True)
            for _addr in address:
                if _addr in connected_devices:
                    self.address = _addr
                    break
            else:
                self.address = address[0]
        else:
            self.address = address

    @retry(RETRY)
    def write_byte(self, data: int) -> bool:
        """ Write a byte to the I2C bus

        Args:
            data (int): byte to write

        Returns:
            bool: True if the byte is written successfully, False otherwise
        """
        self.log.debug(f"write_byte: 0x{data:02x}({data})")
        result = self._smbus.write_byte(self.address, data)
        return result

    @retry(RETRY)
    def write_byte_data(self, reg: int, data: int) -> bool:
        """ Write a byte to the I2C bus

        Args:
            reg (int): register address
            data (int): byte to write

        Returns:
            bool: True if the byte is written successfully, False otherwise
        """
        self.log.debug(f"write_byte_data:\n  address: 0x{self.address:02x}\n      reg: 0x{reg:02x}({reg})\n     data: 0x{data:02x}({data})")
        result = self._smbus.write_byte_data(self.address, reg, data)
        return result

    @retry(RETRY)
    def write_word_data(self, reg: int, data: int, lsb: bool = False) -> bool:
        """ Write a word to the I2C bus

        Args:
            reg (int): register address
            data (int): word to write
            lsb (bool, optional): True if the word is written in little-endian, False otherwise, default is False

        Returns:
            bool: True if the word is written successfully, False otherwise
        """
        msg = f"write_word_data: 0x{reg:02x}({reg}), 0x{data:04x}({data})"
        if lsb:
            l_byte = (data >> 0) & 0xFF
            h_byte = (data >> 8) & 0xFF
            data = (l_byte << 8) | h_byte
            msg += f", LSB={lsb} (sent: 0x{data:04x}({data}))"
        self.log.debug(msg)
        return self._smbus.write_word_data(self.address, reg, data)

    @retry(RETRY)
    def write_i2c_block_data(self, reg: int, data: list) -> bool:
        """ Write a block of data to the I2C bus

        Args:
            reg (int): register address
            data (list): block of data to write

        Returns:
            bool: True if the block of data is written successfully, False otherwise
        """
        self.log.debug(f"write_i2c_block_data: 0x{reg:02x}({reg}), {data}")
        result = self._smbus.write_i2c_block_data(self.address, reg, data)
        return result

    @retry(RETRY)
    def read_byte(self) -> int:
        """ Read a byte from the I2C bus

        Returns:
            int: byte read from the I2C bus
        """
        msg = f"read_byte: 0x{self.address:02x}({self.address})"
        result = self._smbus.read_byte(self.address)
        msg += f" -> 0x{result:02x}({result})"
        self.log.debug(msg)
        return result

    @retry(RETRY)
    def read_byte_data(self, reg: int) -> int:
        """ Read a byte from the I2C bus

        Args:
            reg (int): register address

        Returns:
            int: byte read from the I2C bus
        """
        msg = f"read_byte_data: 0x{reg:02x}({reg})"
        result = self._smbus.read_byte_data(self.address, reg)
        msg += f" -> 0x{result:02x}({result})"
        self.log.debug(msg)
        return result

    @retry(RETRY)
    def read_word_data(self, reg: int, lsb: bool = False) -> int:
        """ Read a word from the I2C bus

        Args:
            reg (int): register address
            lsb (bool, optional): True if the word is read in little-endian, False otherwise, default is False

        Returns:
            int: word read from the I2C bus
        """
        msg = f"read_word_data: 0x{reg:02x}({reg})"
        result = self._smbus.read_word_data(self.address, reg)
        if lsb:
            l_byte = (result >> 0) & 0xFF
            h_byte = (result >> 8) & 0xFF
            result = (l_byte << 8) | h_byte  
            msg += f", LSB={lsb}"
        msg += f" -> 0x{result:04x}({result})"
        self.log.debug(msg)
        return result

    @retry(RETRY)
    def read_i2c_block_data(self, reg: int, num: int) -> list:
        """ Read a block of data from the I2C bus

        Args:
            reg (int): register address
            num (int): number of bytes to read

        Returns:
            list: block of data read from the I2C bus
        """
        msg = f"read_i2c_block_data: 0x{reg:02x}, {num} bytes"
        result = self._smbus.read_i2c_block_data(self.address, reg, num)
        msg += f" -> {result}"
        self.log.debug(msg) 
        return result

    @retry(RETRY)
    def is_ready(self) -> bool:
        """Check if the I2C device is ready

        Returns:
            bool: True if the I2C device is ready, False otherwise
        """
        self.log.debug(f"Check if 0x{self.address:02x}({self.address}) is ready")
        addresses = self.scan(all=True)
        if self.address in addresses:
            self.log.debug(f"0x{self.address:02x}({self.address}) is ready")
            return True
        else:
            self.log.debug(f"0x{self.address:02x}({self.address}) is not ready")
            return False

    @staticmethod
    def scan(bus: int = 1, force: bool = False, search: list = None, all: bool = False) -> list:
        """Scan the I2C bus for devices

        Args:
            bus (int, optional): I2C bus number, default is 1
            force (bool, optional): True if force to access the I2C bus, False otherwise, default is False
            search (list, optional): List of I2C addresses to search, default is None
            all (bool, optional): True if scan all I2C addresses in range 0x00-0x7F, False otherwise, default is False

        Returns:
            list: List of I2C addresses of devices found
        """
        devices = []
        MIN = 0x00 if all else 0x08
        MAX = 0x7F if all else 0x77

        for addr in range(MIN, MAX + 1):
            try:
                with SMBus(bus) as smbus:
                    # Write a empty byte to the address
                    msg = i2c_msg.write(addr, [])
                    smbus.i2c_rdwr(msg)
                    # If no exception, the address is valid
                    if search is None or addr in search:
                        devices.append(addr)
            except OSError as expt:
                # Ignore device busy or unresponsive errors
                if expt.errno == 16:  # Device or resource busy
                    # print(f"Address 0x{addr:02X} busy")
                    pass
                # Other errors continue to try
                continue
        return devices

    def is_avaliable(self) -> bool:
        """ Check if the I2C device is avaliable

        Returns:
            bool: True if the I2C device is avaliable, False otherwise
        """
        return self.address in self.scan()
