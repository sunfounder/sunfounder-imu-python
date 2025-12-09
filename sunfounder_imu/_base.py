import logging
from ._logger import Logger
from ._config import Config

class _Base:
    """ Base class for Fusion Hat

    To implement for all class

    - log: Logger object for logging

    Args:
        log (logging.Logger): Logger, default is None
        log_level (int, str, optional): Log level, default is logging.INFO
        config_file (str, optional): Config file path, default is None
    """
    DEFAULT_CONFIG_FILE = "~/.config/sunfounder-imu-config.json"
    def __init__(self, *args,
            log: logging.Logger = Logger(__name__),
            log_level: [int, str] = logging.INFO,
            config_file: str = DEFAULT_CONFIG_FILE,
            **kwargs):

        self.log = log
        self.config = Config(config_file)
        self.log.setLevel(log_level)
