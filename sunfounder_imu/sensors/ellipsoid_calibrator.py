import numpy as np

DEFAULT_BIAS = [0, 0, 0]
DEFAULT_S_INV = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
]

class EllipsoidCalibrator:
    """Ellipsoid Calibrator
    
    Args:
        ref_norm (float): reference norm for calibration
        bias (list, optional): initial bias. Defaults to None.
        s_inv (list, optional): initial sensitivity inverse. Defaults to None.
    """
    def __init__(self, ref_norm: float=1, bias: list=DEFAULT_BIAS, s_inv: list=DEFAULT_S_INV):
        """
        初始化校准器
        :param ref_norm: 校准目标的参考模长（加速度计9.81，地磁≈50，陀螺仪0）
        """
        self.ref_norm = ref_norm # reference norm for calibration
        self.set_bias(bias)
        self.set_sensitivity_inverse(s_inv)

    def set_bias(self, bias: list):
        """Set bias
        
        Args:
            bias (list): bias vector (3x1)
        """
        self.B = np.array(bias)

    def set_sensitivity(self, sensitivity: list):
        """Set sensitivity
        
        Args:
            sensitivity (list): sensitivity matrix (3x3)
        """
        self.S = np.array(sensitivity)
        self.S_inv = np.linalg.inv(self.S)

    def set_sensitivity_inverse(self, s_inv: list):
        """Set sensitivity inverse
        
        Args:
            s_inv (list): sensitivity inverse matrix (3x3)
        """
        self.S_inv = np.array(s_inv)

    def fit(self, raw_data) -> tuple:
        """Ellipsoid Fitting

        Args:
            raw_data (np.array): N×3 list of raw data (N≥6, each row is a mean of static pose)
            
        Returns:
            tuple: (bias, sensitivity_inv)
        """
        raw_data = np.array(raw_data)
        if raw_data.shape[0] < 6:
            raise ValueError("Data size must be at least 6!")
        if raw_data.shape[1] != 3:
            raise ValueError("Data format error! Should be N×3 2D array (3-axis data)")

        # Special handling for gyroscope calibration (ref_norm=0)
        if self.ref_norm == 0:
            # Simple zero bias calibration for gyroscope
            self.B = np.round(np.mean(raw_data, axis=0), 2)
            self.S = np.eye(3)  # Identity matrix for sensitivity (no scaling)
            self.S_inv = np.eye(3)  # Inverse of identity is identity
            return self.B.tolist(), self.S_inv.tolist()

        # 1. construct linear regression matrix A
        N = raw_data.shape[0]
        A = np.zeros((N, 10))
        A[:, 0] = raw_data[:, 0] ** 2  # x²
        A[:, 1] = raw_data[:, 1] ** 2  # y²
        A[:, 2] = raw_data[:, 2] ** 2  # z²
        A[:, 3] = 2 * raw_data[:, 0] * raw_data[:, 1]  # 2xy
        A[:, 4] = 2 * raw_data[:, 0] * raw_data[:, 2]  # 2xz
        A[:, 5] = 2 * raw_data[:, 1] * raw_data[:, 2]  # 2yz
        A[:, 6] = 2 * raw_data[:, 0]  # 2x
        A[:, 7] = 2 * raw_data[:, 1]  # 2y
        A[:, 8] = 2 * raw_data[:, 2]  # 2z
        A[:, 9] = np.ones(N)  # constant term

        # # 2. least squares to fit ellipsoid parameters
        # def residuals(p):
        #     return A @ p

        # # initial guess: ideal ellipsoid (no error)
        # p0 = np.array([1, 1, 1, 0, 0, 0])
        # result = least_squares(residuals, p0, method='lm')
        # p = result.x / np.linalg.norm(result.x)  # normalize parameters

        # 2. Solve linear system A*p = 0 using SVD decomposition
        # This avoids the trivial zero solution by finding the smallest singular value
        u, s, vh = np.linalg.svd(A)
        # The solution is the right singular vector corresponding to the smallest singular value
        p = vh[-1, :]

        # 3. extract calibration parameters
        # Form the quadratic form matrix S
        S = np.array([
            [p[0], p[3], p[4]],
            [p[3], p[1], p[5]],
            [p[4], p[5], p[2]]
        ])
        
        # Extract bias vector B
        B = np.array([p[6], p[7], p[8]])
        
        # Compute the center of the ellipsoid (actual bias)
        # The center is given by -S_inv * b, where b = [p[6], p[7], p[8]]
        try:
            S_inv = np.linalg.inv(S)
            self.B = -np.dot(S_inv, B)
        except np.linalg.LinAlgError:
            raise ValueError("Singular matrix encountered during calibration. Please provide more diverse calibration data.")
        
        # Compute the actual sensitivity matrix by scaling S to match the reference norm
        # Calculate the radius of the ellipsoid at the center
        radius = np.sqrt(np.dot(np.dot(self.B, S), self.B) - p[9])
        
        # Scale S to achieve the desired reference norm
        if radius > 0:
            scaling_factor = self.ref_norm / radius
        else:
            scaling_factor = 1.0
        
        # Ensure S is positive definite by checking the sign
        # Use the first data point to determine the correct sign
        if raw_data.shape[0] > 0:
            test_point = raw_data[0]
            val = np.dot(np.dot((test_point - self.B), S), (test_point - self.B))
            if val < 0:
                scaling_factor *= -1
        
        # Apply scaling
        self.S = S * scaling_factor
        
        # Ensure S_inv calculation is correct - avoid division by scaling factor
        try:
            self.S_inv = np.linalg.inv(self.S)
        except np.linalg.LinAlgError:
            raise ValueError("Singular matrix encountered when calculating S_inv. Please provide more diverse calibration data.")
        
        # Round to 2 decimal places
        self.B = np.round(self.B, 2)
        self.S = np.round(self.S, 2)
        self.S_inv = np.round(self.S_inv, 2)

        return self.B.tolist(), self.S_inv.tolist()

    def calibrate(self, raw_data):
        """ Calibrate raw data
        
        Args:
            raw_data (np.array): N×3 list of raw data (3-axis data)
            
        Returns:
            np.array: N×3 list of calibrated data (3-axis data)
        """
        
        raw_data = np.array(raw_data)
        # handle single frame data (1×3) or multiple frames (N×3)
        if len(raw_data.shape) == 1:
            raw_data = raw_data.reshape(1, 3)
        
        # Core calibration formula: calibrated = S_inv × (raw_data - B)
        calibrated = np.dot(self.S_inv, (raw_data - self.B).T).T
        
        # Gyroscope special handling: static zero bias calibration should be close to 0, return raw_data - B directly
        if self.ref_norm == 0:
            calibrated = raw_data - self.B
        
        # Ensure output dimension matches input (1×3 for single frame, N×3 for multiple frames)
        return calibrated.squeeze().tolist()
