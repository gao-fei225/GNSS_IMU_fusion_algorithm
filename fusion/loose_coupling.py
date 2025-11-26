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
    
    def process_gnss_measurement(self, nominal_state, gnss_nav):
        """
        处理GNSS导航解量测
        
        Args:
            nominal_state: 当前名义状态
            gnss_nav: GNSS导航解（位置、速度）
        
        Returns:
            修正后的名义状态
        """
        # TODO: 计算量测新息
        # TODO: 计算H矩阵和R矩阵
        # TODO: 执行更新
        # TODO: 修正名义状态
        pass
