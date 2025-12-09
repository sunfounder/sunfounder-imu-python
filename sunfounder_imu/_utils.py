#!/usr/bin/env python3
from typing import Callable, Any

def mapping(x: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:   
    """ Map value from one range to another range

    Args:
        x (float): value to map
        in_min (float): input minimum
        in_max (float): input maximum
        out_min (float): output minimum
        out_max (float): output maximum

    Returns:
        float: mapped value
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def retry(times: int = 5):
    """ Retry decorator retry specified times if any error occurs

    Args:
        times (int, optional): number of times to retry. Defaults to 5.

    Returns:
        function: wrapper function
    """
    def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
        def wrapper(*arg, **kwargs):
            for _ in range(times):
                try:
                    return func(*arg, **kwargs)
                except OSError:
                    continue
            else:
                return False

        return wrapper
    return decorator

def twos_complement(val, bits):
    """ Convert a two's complement value to a signed integer.

    Args:
        val (int): The two's complement value to convert.
        bits (int): The number of bits used to represent the value.

    Returns:
        int: The signed integer value.
    """
    # Get the mask to keep only the lower bits, ensuring val only retains bits bits (handles cases where val exceeds bit width)
    mask = (1 << bits) - 1  # Generate a mask with bits bits set to 1 (e.g., 24 bits → 0xFFFFFF)
    val = val & mask        # Keep only the lower bits, discarding the higher bits
    
    # Convert to signed integer (core logic remains the same)
    if val & (1 << (bits - 1)):
        val -= (1 << bits)
    return val