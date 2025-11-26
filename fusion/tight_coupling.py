"""
紧耦合ESKF
使用GNSS原始观测（伪距、伪距率）进行融合
"""

from .eskf import ESKF


class TightCouplingESKF(ESKF):
    """
    紧耦合ESKF
    
    量测：GNSS原始观测（伪距、伪距率/多普勒）
    
    流程（每个时间步）：
    1. 预测：使用IMU数据推进协方差
    2. 判断是否有GNSS原始观测且未被遮挡
    3. 若有，对每颗卫星计算伪距和伪距率残差
    4. 构建量测矩阵H（线性化）
    5. 执行更新步骤
    6. 修正名义状态，重置误差状态
    
    优势：
    - 可以使用部分卫星观测（不需要完整导航解）
    - 对GNSS遮挡更鲁棒
    - 可以更好地利用多源信息
    """
    
    def __init__(self, initial_cov, error_dynamics, measurement_model):
        """
        初始化紧耦合ESKF
        
        Args:
            initial_cov: 初始协方差矩阵
            error_dynamics: 误差动力学模型
            measurement_model: 紧耦合量测模型
        """
        super().__init__(initial_cov, error_dynamics)
        self.measurement_model = measurement_model
    
    def process_gnss_measurement(self, nominal_state, gnss_raw):
        """
        处理GNSS原始观测
        
        Args:
            nominal_state: 当前名义状态
            gnss_raw: GNSS原始观测（satellites列表）
        
        Returns:
            修正后的名义状态
        """
        # TODO: 对每颗卫星计算量测新息
        # TODO: 构建H矩阵和R矩阵
        # TODO: 执行更新
        # TODO: 修正名义状态
        pass
