"""
误差状态定义
定义ESKF中的误差状态向量
"""

import numpy as np


class ErrorState:
    """
    误差状态类
    
    15维误差状态向量：
    - δθ (3): 姿态误差（rad）
    - δv (3): 速度误差（m/s）
    - δp (3): 位置误差（m）
    - δbg (3): 陀螺零偏误差（rad/s）
    - δba (3): 加计零偏误差（m/s²）
    
    每个误差分量的物理含义：
    - 姿态误差：由陀螺噪声和零偏引起的姿态偏差
    - 速度误差：姿态误差和加计误差耦合导致的速度偏差
    - 位置误差：速度误差积分导致的位置偏差
    - 陀螺零偏：随机游走过程
    - 加计零偏：随机游走过程
    """
    
    def __init__(self):
        """
        初始化误差状态
        """
        self.state = np.zeros(15)
    
    @property
    def attitude_error(self):
        """姿态误差 δθ"""
        return self.state[0:3]
    
    @attitude_error.setter
    def attitude_error(self, value):
        self.state[0:3] = value
    
    @property
    def velocity_error(self):
        """速度误差 δv"""
        return self.state[3:6]
    
    @velocity_error.setter
    def velocity_error(self, value):
        self.state[3:6] = value
    
    @property
    def position_error(self):
        """位置误差 δp"""
        return self.state[6:9]
    
    @position_error.setter
    def position_error(self, value):
        self.state[6:9] = value
    
    @property
    def gyro_bias_error(self):
        """陀螺零偏误差 δbg"""
        return self.state[9:12]
    
    @gyro_bias_error.setter
    def gyro_bias_error(self, value):
        self.state[9:12] = value
    
    @property
    def accel_bias_error(self):
        """加计零偏误差 δba"""
        return self.state[12:15]
    
    @accel_bias_error.setter
    def accel_bias_error(self, value):
        self.state[12:15] = value
    
    def reset(self):
        """
        重置误差状态为零（ESKF标准操作）
        """
        self.state = np.zeros(15)
    
    def get_vector(self):
        """
        获取误差状态向量
        
        Returns:
            np.ndarray: 15维误差状态向量
        """
        return self.state.copy()
    
    def set_vector(self, vector):
        """
        设置误差状态向量
        
        Args:
            vector: 15维误差状态向量
        """
        self.state = vector.copy()
