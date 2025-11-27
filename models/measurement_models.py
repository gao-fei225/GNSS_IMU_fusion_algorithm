"""
量测模型
定义松耦合和紧耦合的量测方程
"""

import numpy as np


class GNSSMeasurementModel:
    """
    GNSS量测模型基类
    """
    
    def compute_innovation(self, nominal_state, measurement):
        """
        计算量测新息（残差）
        
        Args:
            nominal_state: 名义状态
            measurement: GNSS量测
        
        Returns:
            np.ndarray: 量测新息向量
        """
        raise NotImplementedError
    
    def compute_H_matrix(self, nominal_state):
        """
        计算量测矩阵 H
        
        Args:
            nominal_state: 名义状态
        
        Returns:
            np.ndarray: 量测矩阵
        """
        raise NotImplementedError
    
    def compute_R_matrix(self):
        """
        计算量测噪声协方差矩阵 R
        
        Returns:
            np.ndarray: 量测噪声协方差矩阵
        """
        raise NotImplementedError


class LooseCouplingModel(GNSSMeasurementModel):
    """
    松耦合量测模型
    
    量测：GNSS导航解（位置、速度）
    量测残差：z = GNSS导航量 - INS名义导航量
    量测矩阵H：建立残差与误差状态的关系
    
    对于位置量测：
    - z_pos = pos_GNSS - pos_INS
    - z_pos ≈ δp（位置误差）
    - H_pos = [0_{3×3}, 0_{3×3}, I_{3×3}, 0_{3×6}]
    
    对于速度量测：
    - z_vel = vel_GNSS - vel_INS
    - z_vel ≈ δv（速度误差）
    - H_vel = [0_{3×3}, I_{3×3}, 0_{3×9}]
    """
    
    def __init__(self, gnss_params):
        """
        初始化松耦合量测模型
        
        Args:
            gnss_params: GNSS参数字典，包含位置噪声、速度噪声
        """
        self.position_noise = gnss_params['position_noise']
        self.velocity_noise = gnss_params['velocity_noise']
    
    def compute_innovation(self, nominal_state, measurement):
        """
        计算量测新息（GNSS导航解 - INS名义状态）
        
        Args:
            nominal_state: 名义状态，包含position、velocity
            measurement: GNSS导航解，包含position、velocity
        
        Returns:
            np.ndarray: 6维新息向量 [δp(3), δv(3)]
            
        量测新息计算：
        - 位置新息：Δp = pos_GNSS - pos_INS
        - 速度新息：Δv = vel_GNSS - vel_INS
        
        在ESKF框架下，量测新息反映了名义状态的偏差，
        这些偏差应该与误差状态估计相关联。
        """
        # 提取名义状态
        pos_ins = np.array(nominal_state['position'])
        vel_ins = np.array(nominal_state['velocity'])
        
        # 提取GNSS量测
        pos_gnss = np.array(measurement['position'])
        vel_gnss = np.array(measurement['velocity'])
        
        # 计算新息（GNSS - INS）
        innovation_pos = pos_gnss - pos_ins
        innovation_vel = vel_gnss - vel_ins
        
        # 组合为6维新息向量
        innovation = np.concatenate([innovation_pos, innovation_vel])
        
        return innovation
    
    def compute_H_matrix(self, nominal_state=None):
        """
        计算量测矩阵 H（6×15）
        
        Args:
            nominal_state: 名义状态（此处不需要，因为H是常矩阵）
        
        Returns:
            np.ndarray: 量测矩阵
            
        量测方程线性化：
        对于位置量测：z_pos = pos_GNSS - pos_INS ≈ δp
        对于速度量测：z_vel = vel_GNSS - vel_INS ≈ δv
        
        因此量测矩阵H为：
        H = [[0_{3×3}, 0_{3×3}, I_{3×3}, 0_{3×3}, 0_{3×3}],  # 位置量测
             [0_{3×3}, I_{3×3}, 0_{3×3}, 0_{3×3}, 0_{3×3}]]  # 速度量测
        
        误差状态：[δθ(3), δv(3), δp(3), δbg(3), δba(3)]
        量测：    [Δp(3), Δv(3)]
        """
        H = np.zeros((6, 15))
        
        # 位置量测：观测位置误差δp（误差状态的第6-8维）
        H[0:3, 6:9] = np.eye(3)
        
        # 速度量测：观测速度误差δv（误差状态的第3-5维）
        H[3:6, 3:6] = np.eye(3)
        
        return H
    
    def compute_R_matrix(self):
        """
        计算量测噪声协方差矩阵 R（6×6）
        
        Returns:
            np.ndarray: 量测噪声协方差矩阵
            
        量测噪声协方差（对角阵）：
        R = diag([σ_pos²·I(3), σ_vel²·I(3)])
        
        其中：
        - σ_pos: GNSS位置噪声标准差（m）
        - σ_vel: GNSS速度噪声标准差（m/s）
        """
        R = np.zeros((6, 6))
        
        # 位置量测噪声
        R[0:3, 0:3] = np.eye(3) * (self.position_noise ** 2)
        
        # 速度量测噪声
        R[3:6, 3:6] = np.eye(3) * (self.velocity_noise ** 2)
        
        return R


class TightCouplingModel(GNSSMeasurementModel):
    """
    紧耦合量测模型
    
    量测：GNSS原始观测（伪距、伪距率/多普勒）
    
    伪距量测模型：
    - ρ_measured = ||pos_sat - pos_receiver|| + c·dt + noise
    - ρ_predicted = ||pos_sat - pos_INS||
    - innovation = ρ_measured - ρ_predicted
    
    伪距率/多普勒量测模型：
    - ρ̇_measured = (pos_sat - pos_receiver)·(vel_sat - vel_receiver)/||pos_sat - pos_receiver|| + noise
    - ρ̇_predicted = (pos_sat - pos_INS)·(vel_sat - vel_INS)/||pos_sat - pos_INS||
    - innovation = ρ̇_measured - ρ̇_predicted
    
    量测矩阵H：
    - 将位置误差、速度误差线性化到伪距/伪距率残差
    - H的每一行对应一颗卫星的观测
    """
    
    def __init__(self, gnss_params):
        """
        初始化紧耦合量测模型
        
        Args:
            gnss_params: GNSS参数字典，包含伪距噪声、多普勒噪声
        """
        self.pseudorange_noise = gnss_params['pseudorange_noise']
        self.doppler_noise = gnss_params['doppler_noise']
    
    def compute_innovation(self, nominal_state, measurement):
        """
        计算量测新息
        
        Args:
            nominal_state: 名义状态，包含position、velocity
            measurement: GNSS原始观测，包含satellites列表
        
        Returns:
            np.ndarray: n×1新息向量（n为卫星数×2）
        """
        # TODO: 对每颗卫星计算伪距和伪距率残差
        innovations = []
        return np.array(innovations)
    
    def compute_H_matrix(self, nominal_state, measurement):
        """
        计算量测矩阵 H（n×15，n为卫星数×2）
        
        Args:
            nominal_state: 名义状态
            measurement: GNSS原始观测
        
        Returns:
            np.ndarray: 量测矩阵
        """
        # TODO: 对每颗卫星构建量测矩阵的一行（线性化）
        H_rows = []
        return np.array(H_rows)
    
    def compute_R_matrix(self, num_satellites):
        """
        计算量测噪声协方差矩阵 R（n×n，n为卫星数×2）
        
        Args:
            num_satellites: 卫星数量
        
        Returns:
            np.ndarray: 量测噪声协方差矩阵
        """
        # TODO: 构建R矩阵（对角阵）
        n = num_satellites * 2
        R = np.zeros((n, n))
        return R
