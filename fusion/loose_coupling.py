"""
松耦合ESKF
使用GNSS导航解进行融合
"""

from .eskf import ESKF


class LooseCouplingESKF(ESKF):
    """
    松耦合ESKF
    
    量测：GNSS导航解（位置、速度）
    
    流程（每个时间步）：
    1. 预测：使用IMU数据推进协方差
    2. 判断是否有GNSS数据且未被遮挡
    3. 若有，计算量测残差和H矩阵
    4. 执行更新步骤
    5. 修正名义状态，重置误差状态
    """
    
    def __init__(self, initial_cov, error_dynamics, measurement_model):
        """
        初始化松耦合ESKF
        
        Args:
            initial_cov: 初始协方差矩阵
            error_dynamics: 误差动力学模型
            measurement_model: 松耦合量测模型
        """
        super().__init__(initial_cov, error_dynamics)
        self.measurement_model = measurement_model
    
    def process_gnss_measurement(self, nominal_state, gnss_nav, gyro_bias=None, accel_bias=None):
        """
        处理GNSS导航解量测，执行ESKF更新和状态修正
        
        Args:
            nominal_state: 当前名义状态（attitude, velocity, position）
            gnss_nav: GNSS导航解，包含position和velocity
            gyro_bias: 当前陀螺零偏估计（可选）
            accel_bias: 当前加计零偏估计（可选）
        
        Returns:
            tuple: (修正后的名义状态, 修正后的陀螺零偏, 修正后的加计零偏)
            
        流程：
        1. 计算量测新息（GNSS - INS）
        2. 计算量测矩阵H和量测噪声协方差R
        3. 执行ESKF更新步骤
        4. 用误差状态修正名义状态和零偏
        5. 重置误差状态为零
        """
        # 1. 计算量测新息
        innovation = self.measurement_model.compute_innovation(nominal_state, gnss_nav)
        
        # 2. 计算量测矩阵H（6×15）
        H = self.measurement_model.compute_H_matrix(nominal_state)
        
        # 3. 计算量测噪声协方差R（6×6）
        R = self.measurement_model.compute_R_matrix()
        
        # 4. 执行ESKF更新步骤
        self.update(innovation, H, R)
        
        # 5. 修正名义状态和零偏，重置误差状态
        corrected_state, corrected_gyro_bias, corrected_accel_bias = \
            self.correct_nominal_state(nominal_state, gyro_bias, accel_bias)
        
        return corrected_state, corrected_gyro_bias, corrected_accel_bias
