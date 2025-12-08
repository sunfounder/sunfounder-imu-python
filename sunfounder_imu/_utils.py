#!/usr/bin/env python3
import os
import time
from typing import Callable, Any

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
    # 第一步：截断高位，确保val仅保留bits位（处理超出位宽的情况）
    mask = (1 << bits) - 1  # 生成bits位全1的掩码（如24位→0xFFFFFF）
    val = val & mask        # 保留低bits位，丢弃高位
    
    # 第二步：补码转换（核心逻辑不变）
    if val & (1 << (bits - 1)):
        val -= (1 << bits)
    return val