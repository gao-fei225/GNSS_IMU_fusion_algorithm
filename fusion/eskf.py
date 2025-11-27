"""
ESKF基类
实现误差状态卡尔曼滤波的基本框架
"""

import numpy as np


class ESKF:
    """
    误差状态卡尔曼滤波器基类
    
    核心流程：
    1. 预测步骤（Predict）：
       - 根据F、G、Q推进误差状态协方差
       - 误差状态估计保持为零（在预测阶段）
    
    2. 更新步骤（Update）：
       - 计算量测新息（实测 - 预测）
       - 计算卡尔曼增益 K = P·H'·(H·P·H' + R)^(-1)
       - 更新误差状态估计 δx = K·innovation
       - 更新协方差 P = (I - K·H)·P
    
    3. 状态修正（Correction）：
       - 用误差状态修正名义状态
       - 重置误差状态为零（ESKF标准操作）
    """
    
    def __init__(self, initial_cov, error_dynamics):
        """
        初始化ESKF
        
        Args:
            initial_cov: 初始协方差矩阵（15×15）
            error_dynamics: 误差动力学模型
        """
        self.P = initial_cov  # 协方差矩阵
        self.error_state = np.zeros(15)  # 误差状态估计
        self.error_dynamics = error_dynamics
    
    def predict(self, nominal_state, imu_measurement, dt):
        """
        预测步骤：推进协方差矩阵
        
        Args:
            nominal_state: 当前名义状态
            imu_measurement: IMU测量
            dt: 时间间隔
            
        在ESKF中，预测步骤只需要更新协方差矩阵P，
        误差状态估计保持为零（因为误差已经在上一次更新后修正到名义状态中）
        
        步骤：
        1. 计算状态转移矩阵F和噪声传递矩阵G
        2. 计算过程噪声协方差矩阵Q
        3. 离散化：得到Φ和Qd
        4. 更新协方差：P = Φ·P·Φ^T + Qd
        """
        # 计算F矩阵
        F = self.error_dynamics.compute_F_matrix(nominal_state, imu_measurement)
        
        # 计算G矩阵
        G = self.error_dynamics.compute_G_matrix(nominal_state)
        
        # 计算Q矩阵
        Q = self.error_dynamics.compute_Q_matrix(dt)
        
        # 离散化
        Phi, Qd = self.error_dynamics.discretize(F, G, Q, dt)
        
        # 更新协方差矩阵
        self.P = Phi @ self.P @ Phi.T + Qd
    
    def update(self, innovation, H, R):
        """
        更新步骤：使用量测修正误差状态和协方差
        
        Args:
            innovation: 量测新息（实测 - 预测）
            H: 量测矩阵（量测与误差状态的线性关系）
            R: 量测噪声协方差矩阵
            
        卡尔曼滤波更新步骤：
        1. 计算新息协方差：S = H·P·H^T + R
        2. 计算卡尔曼增益：K = P·H^T·S^(-1)
        3. 更新误差状态估计：δx = δx + K·innovation
        4. 更新协方差：P = (I - K·H)·P
        """
        # 计算新息协方差
        S = H @ self.P @ H.T + R
        
        # 计算卡尔曼增益
        # K = P·H^T·S^(-1)
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # 更新误差状态估计
        self.error_state = self.error_state + K @ innovation
        
        # 更新协方差矩阵（使用Joseph形式保证数值稳定性）
        # P = (I - K·H)·P·(I - K·H)^T + K·R·K^T
        # 简化形式：P = (I - K·H)·P
        I_KH = np.eye(15) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
    
    def correct_nominal_state(self, nominal_state, gyro_bias=None, accel_bias=None):
        """
        用误差状态修正名义状态，然后重置误差状态
        
        Args:
            nominal_state: 要修正的名义状态（包含attitude, velocity, position）
            gyro_bias: 当前陀螺零偏估计（如果为None则不修正）
            accel_bias: 当前加计零偏估计（如果为None则不修正）
        
        Returns:
            tuple: (修正后的名义状态, 修正后的陀螺零偏, 修正后的加计零偏)
            
        修正步骤：
        1. 姿态修正：使用小角度近似，将姿态误差δθ转换为四元数增量，
           然后与名义姿态四元数相乘
        2. 速度修正：直接加上速度误差
        3. 位置修正：直接加上位置误差
        4. 零偏修正：更新零偏估计
        5. 重置误差状态为零（ESKF标准操作）
        """
        # 提取误差状态各分量
        delta_theta = self.error_state[0:3]   # 姿态误差
        delta_v = self.error_state[3:6]       # 速度误差
        delta_p = self.error_state[6:9]       # 位置误差
        delta_bg = self.error_state[9:12]     # 陀螺零偏误差
        delta_ba = self.error_state[12:15]    # 加计零偏误差
        
        # 复制名义状态（避免修改原始数据）
        corrected_state = {
            'attitude': nominal_state['attitude'].copy(),
            'velocity': nominal_state['velocity'].copy(),
            'position': nominal_state['position'].copy()
        }
        
        # 1. 姿态修正：将小角度误差转换为四元数增量
        # 对于小角度δθ，四元数增量近似为：δq ≈ [1, δθ/2]
        delta_q = self._error_to_quaternion(delta_theta)
        corrected_state['attitude'] = self._quaternion_multiply(
            corrected_state['attitude'], delta_q
        )
        # 归一化四元数
        corrected_state['attitude'] = corrected_state['attitude'] / np.linalg.norm(corrected_state['attitude'])
        
        # 2. 速度修正
        corrected_state['velocity'] = corrected_state['velocity'] + delta_v
        
        # 3. 位置修正
        corrected_state['position'] = corrected_state['position'] + delta_p
        
        # 4. 零偏修正
        if gyro_bias is not None:
            corrected_gyro_bias = gyro_bias + delta_bg
        else:
            corrected_gyro_bias = delta_bg.copy()
            
        if accel_bias is not None:
            corrected_accel_bias = accel_bias + delta_ba
        else:
            corrected_accel_bias = delta_ba.copy()
        
        # 5. 重置误差状态为零（ESKF的核心特点）
        self.error_state = np.zeros(15)
        
        return corrected_state, corrected_gyro_bias, corrected_accel_bias
    
    def get_covariance(self):
        """
        获取当前协方差矩阵
        
        Returns:
            np.ndarray: 15×15协方差矩阵
        """
        return self.P.copy()
    
    def get_error_state(self):
        """
        获取当前误差状态估计
        
        Returns:
            np.ndarray: 15维误差状态向量
        """
        return self.error_state.copy()
    
    @staticmethod
    def _error_to_quaternion(error_rotation):
        """
        将小角度旋转误差转换为四元数增量
        
        对于小角度δθ = [δθx, δθy, δθz]^T，对应的四元数增量为：
        δq ≈ [1, δθx/2, δθy/2, δθz/2]^T
        
        这是基于小角度近似：
        - cos(||δθ||/2) ≈ 1
        - sin(||δθ||/2) ≈ ||δθ||/2
        - δθ/||δθ|| ≈ δθ（当δθ很小时）
        
        Args:
            error_rotation: 3维旋转误差向量（弧度）
        
        Returns:
            np.ndarray: 4维四元数增量 [qw, qx, qy, qz]
        """
        # 小角度近似
        norm = np.linalg.norm(error_rotation)
        if norm < 1e-8:
            # 如果误差极小，直接返回单位四元数
            return np.array([1.0, 0.0, 0.0, 0.0])
        
        # 对于小角度，使用一阶近似
        delta_q = np.zeros(4)
        delta_q[0] = 1.0  # qw ≈ 1
        delta_q[1:4] = error_rotation * 0.5  # [qx, qy, qz] ≈ δθ/2
        
        # 归一化
        delta_q = delta_q / np.linalg.norm(delta_q)
        
        return delta_q
    
    @staticmethod
    def _quaternion_multiply(q1, q2):
        """
        四元数乘法
        
        q1 * q2 表示先进行q2旋转，再进行q1旋转
        
        四元数乘法公式：
        q1 * q2 = [q1w*q2w - q1v·q2v, q1w*q2v + q2w*q1v + q1v×q2v]
        
        其中 q1 = [q1w, q1v], q2 = [q2w, q2v]
        
        Args:
            q1: 四元数1 [qw, qx, qy, qz]
            q2: 四元数2 [qw, qx, qy, qz]
        
        Returns:
            np.ndarray: 四元数乘积 [qw, qx, qy, qz]
        """
        q1w, q1x, q1y, q1z = q1
        q2w, q2x, q2y, q2z = q2
        
        qw = q1w*q2w - q1x*q2x - q1y*q2y - q1z*q2z
        qx = q1w*q2x + q1x*q2w + q1y*q2z - q1z*q2y
        qy = q1w*q2y - q1x*q2z + q1y*q2w + q1z*q2x
        qz = q1w*q2z + q1x*q2y - q1y*q2x + q1z*q2w
        
        return np.array([qw, qx, qy, qz])
