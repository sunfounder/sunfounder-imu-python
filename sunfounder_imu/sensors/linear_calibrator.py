
DEFAULT_BIAS = [0.0, 0.0, 0.0]
DEFAULT_SCALE = [1.0, 1.0, 1.0]

class LinearCalibrator:
    """Linear calibrator
    
    Args:
        bias (list[float]): Bias.
        scale (list[float]): Scale.
    """
    def __init__(self, bias: list[float], scale: list[float]) -> None:
        self.bias = bias
        self.scale = scale

    def fit(self, data: list[list[float]]) -> None:
        """Fit the calibrator to the data.
        
        Args:
            data (list[list[float]]): Data to fit.
        """
        x_max, x_min = max(data, key=lambda x: x[0])[0], min(data, key=lambda x: x[0])[0]
        y_max, y_min = max(data, key=lambda x: x[1])[1], min(data, key=lambda x: x[1])[1]
        z_max, z_min = max(data, key=lambda x: x[2])[2], min(data, key=lambda x: x[2])[2]
        self.bias[0] = (x_max + x_min) / 2
        self.bias[1] = (y_max + y_min) / 2
        self.bias[2] = (z_max + z_min) / 2
        self.scale[0] = 2 / (x_max - x_min)
        self.scale[1] = 2 / (y_max - y_min)
        self.scale[2] = 2 / (z_max - z_min)

        return self.bias, self.scale

    def calibrate(self, data) -> tuple[float, float, float]:
        """Calibrate the data.
        
        Args:
            data (list[float]): Data to calibrate.
        
        Returns:
            tuple[float, float, float]: Calibrated x, y, z values.
        """
        x, y, z = data
        x = (x - self.bias[0]) * self.scale[0]
        y = (y - self.bias[1]) * self.scale[1]
        z = (z - self.bias[2]) * self.scale[2]
        return x, y, z

    def set_bias(self, bias: list[float]) -> None:
        """Set the bias.
        
        Args:
            bias (list[float]): Bias.
        """
        self.bias = bias

    def set_scale(self, scale: list[float]) -> None:
        """Set the scale.
        
        Args:
            scale (list[float]): Scale.
        """
        self.scale = scale
