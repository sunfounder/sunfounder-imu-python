#!/usr/bin/env python3
from typing import Callable, Any
import numpy as np

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

def remove_outliers_3d_and_mean(data_3d, sigma=3) -> tuple:
    """ Remove outliers from 3D data and calculate mean
    
    Args:
        data_3d (list): input 3D data, N×3 array (each row is [x, y, z])
        sigma (float, optional): sigma coefficient for outlier removal, default is 3 (adjust to 2 for larger noise)
        
    Returns:
        - axis_means: 1×3 array (mean values after outlier removal for X/Y/Z axes)
    """
    # 1. 数据格式转换与合法性校验
    data_3d = np.array(data_3d, dtype=np.float64)
    if len(data_3d.shape) != 2 or data_3d.shape[1] != 3:
        raise ValueError("输入数据必须是N×3的二维数组（每行是X/Y/Z三轴数据）！")
    if data_3d.shape[0] == 0:
        raise ValueError("输入数据不能为空！")
    if data_3d.shape[0] == 1:
        # 只有1帧数据时，无法计算标准差，直接返回该帧均值和原数据
        axis_means = data_3d[0].copy()
        return axis_means, data_3d.tolist()
    
    # 2. 分别提取X/Y/Z轴的一维数据
    x_data = data_3d[:, 0]
    y_data = data_3d[:, 1]
    z_data = data_3d[:, 2]
    
    # 3. 定义单轴3σ剔除函数（内部辅助函数）
    def _single_axis_filter(data):
        mean_raw = np.mean(data)
        std_raw = np.std(data, ddof=0)  # 总体标准差（传感器采样用总体标准差更合适）
        # 计算3σ上下限
        lower = mean_raw - sigma * std_raw
        upper = mean_raw + sigma * std_raw
        # 剔除异常值
        filtered = data[(data >= lower) & (data <= upper)]
        # 边界处理：若剔除后无数据，返回原始数据（避免空数组）
        if len(filtered) == 0:
            print(f"警告：某轴所有数据被判定为异常值，保留原始数据")
            return data, mean_raw
        return filtered, np.mean(filtered)
    
    # 4. 对X/Y/Z轴分别做3σ剔除并计算均值
    x_filtered, x_mean = _single_axis_filter(x_data)
    y_filtered, y_mean = _single_axis_filter(y_data)
    z_filtered, z_mean = _single_axis_filter(z_data)
    
    # 5. 重构剔除异常值后的完整3轴数据（关键：保证X/Y/Z轴的帧对应）
    # 思路：先找到每帧是否在三个轴的正常范围内，仅保留所有轴都正常的帧
    # 计算各轴的正常范围
    x_mean_raw = np.mean(x_data)
    x_std_raw = np.std(x_data, ddof=0)
    x_lower = x_mean_raw - sigma * x_std_raw
    x_upper = x_mean_raw + sigma * x_std_raw
    
    y_mean_raw = np.mean(y_data)
    y_std_raw = np.std(y_data, ddof=0)
    y_lower = y_mean_raw - sigma * y_std_raw
    y_upper = y_mean_raw + sigma * y_std_raw
    
    z_mean_raw = np.mean(z_data)
    z_std_raw = np.std(z_data, ddof=0)
    z_lower = z_mean_raw - sigma * z_std_raw
    z_upper = z_mean_raw + sigma * z_std_raw
    
    # 保留所有轴都在正常范围的帧
    mask = (
        (x_data >= x_lower) & (x_data <= x_upper) &
        (y_data >= y_lower) & (y_data <= y_upper) &
        (z_data >= z_lower) & (z_data <= z_upper)
    )
    filtered_data_3d = data_3d[mask].tolist()
    
    # 6. 组装三轴均值
    axis_means = np.array([x_mean, y_mean, z_mean])
    
    return axis_means

def format_3d_data(data_3d: list) -> None:
    """ Format 3D data
    
    Args:
        data_3d (list): [x, y, z]
    """
    return f"{data_3d[0]:5.2f}, {data_3d[1]:5.2f}, {data_3d[2]:5.2f}"
