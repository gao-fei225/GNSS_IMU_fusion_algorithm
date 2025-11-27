"""
误差动力学模型
定义误差状态的传播方程（F、G矩阵）
"""

import numpy as np


class ErrorDynamics:
    """
    误差动力学类
    
    负责：
    - 构建误差状态的连续时间微分方程
    - 构造状态转移矩阵 F
    - 构造噪声传递矩阵 G
    - 构造过程噪声协方差矩阵 Q
    - 离散化误差方程
    
    误差状态微分方程：
    dδx/dt = F·δx + G·w
    
    其中：
    - δx: 15维误差状态
    - F: 状态转移矩阵（15×15）
    - G: 噪声传递矩阵（15×12）
    - w: 过程噪声（陀螺噪声、加计噪声、陀螺零偏游走、加计零偏游走）
    """
    
    def __init__(self, imu_params):
        """
        初始化误差动力学模型
        
        Args:
            imu_params: IMU参数字典，包含噪声水平和零偏随机游走
        """
        self.gyro_noise = imu_params['gyro_noise']
        self.accel_noise = imu_params['accel_noise']
        self.gyro_bias_walk = imu_params['gyro_bias_walk']
        self.accel_bias_walk = imu_params['accel_bias_walk']
    
    def compute_F_matrix(self, nominal_state, imu_measurement):
        """
        计算状态转移矩阵 F
        
        Args:
            nominal_state: 当前名义状态（姿态、速度、位置）
            imu_measurement: 当前IMU测量（角速度、加速度）
        
        Returns:
            np.ndarray: 15×15状态转移矩阵
            
        误差状态微分方程：dδx/dt = F·δx + G·w
        
        误差状态向量：δx = [δθ(3), δv(3), δp(3), δbg(3), δba(3)]^T
        
        F矩阵结构（按块划分）：
        ┌─────────────────────────────────────────┐
        │  -[ω]×   0      0     -C_b^n    0      │  δθ  姿态误差
        │ -C_b^n[f]×  0   0       0    -C_b^n    │  δv  速度误差
        │    0      I     0       0       0      │  δp  位置误差
        │    0      0     0       0       0      │  δbg 陀螺零偏误差
        │    0      0     0       0       0      │  δba 加计零偏误差
        └─────────────────────────────────────────┘
        
        其中：
        - [ω]×: 角速度的反对称矩阵
        - [f]×: 比力的反对称矩阵
        - C_b^n: 机体系到导航系的旋转矩阵
        - I: 3×3单位矩阵
        """
        F = np.zeros((15, 15))
        
        # 提取名义状态
        attitude = nominal_state['attitude']  # 四元数 [qw, qx, qy, qz]
        
        # 提取IMU测量
        gyro = np.array(imu_measurement['gyro'])     # 角速度 [wx, wy, wz]
        accel = np.array(imu_measurement['accel'])   # 加速度 [ax, ay, az]
        
        # 计算旋转矩阵（机体系到导航系）
        C_b_n = self._quaternion_to_rotation_matrix(attitude)
        
        # 构建角速度的反对称矩阵 [ω]×
        omega_skew = self._skew_symmetric(gyro)
        
        # 构建比力的反对称矩阵 [f]×
        f_skew = self._skew_symmetric(accel)
        
        # 填充F矩阵各块
        # F11: 姿态误差微分 = -[ω]× · δθ - C_b^n · δbg
        F[0:3, 0:3] = -omega_skew
        F[0:3, 9:12] = -C_b_n
        
        # F21: 速度误差微分 = -C_b^n·[f]× · δθ - C_b^n · δba
        F[3:6, 0:3] = -C_b_n @ f_skew
        F[3:6, 12:15] = -C_b_n
        
        # F31: 位置误差微分 = I · δv
        F[6:9, 3:6] = np.eye(3)
        
        # F41, F51: 零偏误差为随机游走，微分方程为 dδb/dt = 0，对应F矩阵元素为0
        # 已经初始化为零矩阵，无需额外操作
        
        return F
    
    def compute_G_matrix(self, nominal_state):
        """
        计算噪声传递矩阵 G
        
        Args:
            nominal_state: 当前名义状态（需要旋转矩阵）
        
        Returns:
            np.ndarray: 15×12噪声传递矩阵
            
        过程噪声向量：w = [wg(3), wa(3), wbg(3), wba(3)]^T
        - wg: 陀螺白噪声
        - wa: 加计白噪声
        - wbg: 陀螺零偏随机游走噪声
        - wba: 加计零偏随机游走噪声
        
        G矩阵结构：
        ┌─────────────────────────┐
        │  -C_b^n    0     0     0  │  δθ
        │    0    -C_b^n  0     0  │  δv
        │    0       0    0     0  │  δp
        │    0       0    I     0  │  δbg
        │    0       0    0     I  │  δba
        └─────────────────────────┘
        """
        G = np.zeros((15, 12))
        
        # 提取名义状态
        attitude = nominal_state['attitude']
        
        # 计算旋转矩阵（机体系到导航系）
        C_b_n = self._quaternion_to_rotation_matrix(attitude)
        
        # 陀螺白噪声对姿态误差的影响
        G[0:3, 0:3] = -C_b_n
        
        # 加计白噪声对速度误差的影响
        G[3:6, 3:6] = -C_b_n
        
        # 陀螺零偏随机游走对零偏误差的影响
        G[9:12, 6:9] = np.eye(3)
        
        # 加计零偏随机游走对零偏误差的影响
        G[12:15, 9:12] = np.eye(3)
        
        return G
    
    def compute_Q_matrix(self, dt):
        """
        计算过程噪声协方差矩阵 Q
        
        Args:
            dt: 时间间隔（秒）
        
        Returns:
            np.ndarray: 12×12过程噪声协方差矩阵
            
        过程噪声协方差（对角阵）：
        Q = diag([σ_g²·I(3), σ_a²·I(3), σ_bg²·I(3), σ_ba²·I(3)])
        
        其中噪声参数的单位（功率谱密度，PSD）：
        - σ_g: 陀螺白噪声 (rad/s/√Hz)
        - σ_a: 加计白噪声 (m/s²/√Hz)
        - σ_bg: 陀螺零偏随机游走 (rad/s²/√Hz)
        - σ_ba: 加计零偏随机游走 (m/s³/√Hz)
        
        注意：由于是功率谱密度，需要乘以采样时间来得到离散协方差
        但在离散化时会统一处理，这里直接使用PSD的平方
        """
        Q = np.zeros((12, 12))
        
        # 陀螺白噪声协方差 (rad²/s²/Hz)
        Q[0:3, 0:3] = np.eye(3) * (self.gyro_noise ** 2)
        
        # 加计白噪声协方差 (m²/s⁴/Hz)
        Q[3:6, 3:6] = np.eye(3) * (self.accel_noise ** 2)
        
        # 陀螺零偏随机游走协方差 (rad²/s⁴/Hz)
        Q[6:9, 6:9] = np.eye(3) * (self.gyro_bias_walk ** 2)
        
        # 加计零偏随机游走协方差 (m²/s⁶/Hz)
        Q[9:12, 9:12] = np.eye(3) * (self.accel_bias_walk ** 2)
        
        return Q
    
    def discretize(self, F, G, Q, dt):
        """
        离散化误差方程
        
        Args:
            F: 连续时间状态转移矩阵
            G: 噪声传递矩阵
            Q: 过程噪声协方差矩阵
            dt: 时间间隔（秒）
        
        Returns:
            tuple: (Φ, Qd) 离散化的状态转移矩阵和过程噪声协方差
            
        离散化方法：
        1. 状态转移矩阵：一阶近似
           Φ = I + F·dt + 0.5·F²·dt² (可选二阶项以提高精度)
           
        2. 过程噪声协方差：Van Loan方法或一阶近似
           Qd = G·Q·G^T·dt (一阶近似)
           
        对于IMU这样的高采样率系统，一阶近似通常足够精确
        """
        # 状态转移矩阵的一阶离散化
        # Φ ≈ I + F·dt
        Phi = np.eye(15) + F * dt
        
        # 如果需要更高精度，可以加入二阶项
        # Phi = Phi + 0.5 * (F @ F) * (dt ** 2)
        
        # 过程噪声协方差的离散化
        # Qd ≈ G·Q·G^T·dt
        Qd = G @ Q @ G.T * dt
        
        return Phi, Qd
    
    @staticmethod
    def _skew_symmetric(vector):
        """
        构建向量的反对称矩阵（叉乘矩阵）
        
        对于向量 v = [v1, v2, v3]^T，其反对称矩阵为：
        [v]× = [[  0,  -v3,   v2],
                [ v3,    0,  -v1],
                [-v2,   v1,    0]]
        
        满足：[v]× @ u = v × u (叉乘)
        
        Args:
            vector: 3维向量
        
        Returns:
            np.ndarray: 3×3反对称矩阵
        """
        v1, v2, v3 = vector
        return np.array([
            [0,   -v3,  v2],
            [v3,   0,  -v1],
            [-v2,  v1,   0]
        ], dtype=np.float64)
    
    @staticmethod
    def _quaternion_to_rotation_matrix(q):
        """
        将四元数转换为旋转矩阵
        
        Args:
            q: 四元数 [qw, qx, qy, qz]
        
        Returns:
            np.ndarray: 3×3旋转矩阵（机体系到导航系）
        """
        qw, qx, qy, qz = q
        
        C = np.array([
            [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qw*qz),      2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz),      1 - 2*(qx**2 + qz**2),  2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy),      2*(qy*qz + qw*qx),      1 - 2*(qx**2 + qy**2)]
        ], dtype=np.float64)
        
        return C
