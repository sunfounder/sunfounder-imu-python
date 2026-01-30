import logging
from ._logger import Logger
from typing import Optional

class _Base:
    """ Base class for Fusion Hat

    To implement for all class

    - log: Logger object for logging

    Args:
        log (logging.Logger): Logger, default is None
        log_level (int, str, optional): Log level, default is logging.INFO
        config_file (str, optional): Config file path, default is None
    """
    def __init__(self, *args,
            log: Optional[logging.Logger] = None,
            log_level: [int, str] = logging.INFO,
            **kwargs):

        self.log = log if log is not None else Logger(self.__class__.__name__)
        self.log.setLevel(log_level)
