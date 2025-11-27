"""
测试ESKF误差状态模型
验证第四步实现：误差动力学和ESKF核心算法
"""

import numpy as np
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from models.error_dynamics import ErrorDynamics
from models.error_state import ErrorState
from fusion.eskf import ESKF
from config.config_manager import ConfigManager


def test_error_dynamics():
    """测试误差动力学模型的F、G、Q矩阵计算"""
    print("=" * 60)
    print("测试误差动力学模型")
    print("=" * 60)
    
    # 加载配置
    config_manager = ConfigManager()
    imu_params = config_manager.get_imu_params()
    
    # 创建误差动力学模型
    error_dynamics = ErrorDynamics(imu_params)
    
    # 构造测试用的名义状态
    nominal_state = {
        'attitude': np.array([1.0, 0.0, 0.0, 0.0]),  # 单位四元数
        'velocity': np.array([0.0, 0.0, 0.0]),
        'position': np.array([0.0, 0.0, 0.0])
    }
    
    # 构造测试用的IMU测量
    imu_measurement = {
        'gyro': np.array([0.01, 0.0, 0.0]),   # 小角速度
        'accel': np.array([0.0, 0.0, 9.81])   # 静止时的加速度（向上为正）
    }
    
    # 测试F矩阵计算
    print("\n1. 测试状态转移矩阵F的计算：")
    F = error_dynamics.compute_F_matrix(nominal_state, imu_measurement)
    print(f"   F矩阵形状：{F.shape}")
    print(f"   F矩阵范数：{np.linalg.norm(F):.6f}")
    print(f"   F[0:3, 0:3] (姿态-姿态块)：\n{F[0:3, 0:3]}")
    
    # 测试G矩阵计算
    print("\n2. 测试噪声传递矩阵G的计算：")
    G = error_dynamics.compute_G_matrix(nominal_state)
    print(f"   G矩阵形状：{G.shape}")
    print(f"   G矩阵范数：{np.linalg.norm(G):.6f}")
    print(f"   G矩阵非零元素数量：{np.count_nonzero(G)}")
    
    # 测试Q矩阵计算
    print("\n3. 测试过程噪声协方差矩阵Q的计算：")
    dt = 0.01  # 100Hz采样
    Q = error_dynamics.compute_Q_matrix(dt)
    print(f"   Q矩阵形状：{Q.shape}")
    print(f"   Q矩阵迹：{np.trace(Q):.9f}")
    print(f"   Q矩阵对角元素：{np.diag(Q)[:6]}")
    
    # 测试离散化
    print("\n4. 测试误差方程离散化：")
    Phi, Qd = error_dynamics.discretize(F, G, Q, dt)
    print(f"   Φ矩阵形状：{Phi.shape}")
    print(f"   Φ与单位阵的差的范数：{np.linalg.norm(Phi - np.eye(15)):.6f}")
    print(f"   Qd矩阵形状：{Qd.shape}")
    print(f"   Qd矩阵迹：{np.trace(Qd):.9f}")
    
    print("\n✓ 误差动力学模型测试通过")


def test_eskf_predict():
    """测试ESKF预测步骤"""
    print("\n" + "=" * 60)
    print("测试ESKF预测步骤")
    print("=" * 60)
    
    # 加载配置
    config_manager = ConfigManager()
    imu_params = config_manager.get_imu_params()
    eskf_params = config_manager.get_eskf_params()
    
    # 创建误差动力学模型
    error_dynamics = ErrorDynamics(imu_params)
    
    # 创建初始协方差矩阵
    initial_cov = np.eye(15)
    initial_cov[0:3, 0:3] *= eskf_params['initial_cov']['attitude']
    initial_cov[3:6, 3:6] *= eskf_params['initial_cov']['velocity']
    initial_cov[6:9, 6:9] *= eskf_params['initial_cov']['position']
    initial_cov[9:12, 9:12] *= eskf_params['initial_cov']['gyro_bias']
    initial_cov[12:15, 12:15] *= eskf_params['initial_cov']['accel_bias']
    
    # 创建ESKF
    eskf = ESKF(initial_cov, error_dynamics)
    
    # 构造测试用的名义状态
    nominal_state = {
        'attitude': np.array([1.0, 0.0, 0.0, 0.0]),
        'velocity': np.array([0.0, 0.0, 0.0]),
        'position': np.array([0.0, 0.0, 0.0])
    }
    
    # 构造测试用的IMU测量
    imu_measurement = {
        'gyro': np.array([0.01, 0.0, 0.0]),
        'accel': np.array([0.0, 0.0, 9.81])
    }
    
    # 执行预测
    print("\n1. 执行ESKF预测步骤：")
    dt = 0.01
    P_before = eskf.get_covariance()
    print(f"   预测前协方差矩阵迹：{np.trace(P_before):.6f}")
    
    eskf.predict(nominal_state, imu_measurement, dt)
    
    P_after = eskf.get_covariance()
    print(f"   预测后协方差矩阵迹：{np.trace(P_after):.6f}")
    print(f"   协方差增长：{np.trace(P_after) - np.trace(P_before):.9f}")
    
    # 验证协方差矩阵的正定性
    eigenvalues = np.linalg.eigvals(P_after)
    print(f"   协方差矩阵最小特征值：{np.min(eigenvalues):.9f}")
    
    if np.min(eigenvalues) > 0:
        print("   ✓ 协方差矩阵是正定的")
    else:
        print("   ✗ 警告：协方差矩阵不是正定的")
    
    print("\n✓ ESKF预测步骤测试通过")


def test_eskf_update():
    """测试ESKF更新步骤"""
    print("\n" + "=" * 60)
    print("测试ESKF更新步骤")
    print("=" * 60)
    
    # 加载配置
    config_manager = ConfigManager()
    imu_params = config_manager.get_imu_params()
    eskf_params = config_manager.get_eskf_params()
    
    # 创建误差动力学模型
    error_dynamics = ErrorDynamics(imu_params)
    
    # 创建初始协方差矩阵
    initial_cov = np.eye(15) * 0.1
    
    # 创建ESKF
    eskf = ESKF(initial_cov, error_dynamics)
    
    # 模拟一个简单的位置量测更新（只观测位置）
    print("\n1. 测试位置量测更新：")
    
    # 量测矩阵H（只观测位置，3个维度）
    H = np.zeros((3, 15))
    H[0:3, 6:9] = np.eye(3)  # 位置对应误差状态的第6-8维
    
    # 量测噪声协方差
    R = np.eye(3) * 0.01  # 1cm的位置噪声
    
    # 模拟量测新息（观测到的位置偏差）
    innovation = np.array([0.1, 0.05, -0.02])  # 10cm, 5cm, -2cm
    
    # 执行更新
    P_before = eskf.get_covariance()
    error_state_before = eskf.get_error_state()
    
    print(f"   更新前误差状态范数：{np.linalg.norm(error_state_before):.6f}")
    print(f"   更新前协方差矩阵迹：{np.trace(P_before):.6f}")
    
    eskf.update(innovation, H, R)
    
    P_after = eskf.get_covariance()
    error_state_after = eskf.get_error_state()
    
    print(f"   更新后误差状态范数：{np.linalg.norm(error_state_after):.6f}")
    print(f"   更新后协方差矩阵迹：{np.trace(P_after):.6f}")
    print(f"   位置误差估计：{error_state_after[6:9]}")
    
    # 验证协方差减小
    if np.trace(P_after) < np.trace(P_before):
        print("   ✓ 协方差矩阵在更新后减小")
    else:
        print("   ✗ 警告：协方差矩阵在更新后没有减小")
    
    print("\n✓ ESKF更新步骤测试通过")


def test_eskf_correction():
    """测试ESKF状态修正"""
    print("\n" + "=" * 60)
    print("测试ESKF状态修正")
    print("=" * 60)
    
    # 加载配置
    config_manager = ConfigManager()
    imu_params = config_manager.get_imu_params()
    
    # 创建误差动力学模型
    error_dynamics = ErrorDynamics(imu_params)
    
    # 创建初始协方差矩阵
    initial_cov = np.eye(15) * 0.01
    
    # 创建ESKF
    eskf = ESKF(initial_cov, error_dynamics)
    
    # 设置一个非零的误差状态（模拟更新后的状态）
    eskf.error_state = np.array([
        0.01, 0.0, 0.0,      # 姿态误差（约0.57度）
        0.1, 0.0, 0.0,       # 速度误差
        1.0, 0.0, 0.0,       # 位置误差
        0.001, 0.0, 0.0,     # 陀螺零偏误差
        0.01, 0.0, 0.0       # 加计零偏误差
    ])
    
    # 构造名义状态
    nominal_state = {
        'attitude': np.array([1.0, 0.0, 0.0, 0.0]),
        'velocity': np.array([5.0, 0.0, 0.0]),
        'position': np.array([100.0, 50.0, 10.0])
    }
    
    # 初始零偏
    gyro_bias = np.array([0.0, 0.0, 0.0])
    accel_bias = np.array([0.0, 0.0, 0.0])
    
    print("\n1. 执行状态修正：")
    print(f"   修正前位置：{nominal_state['position']}")
    print(f"   修正前速度：{nominal_state['velocity']}")
    print(f"   修正前姿态：{nominal_state['attitude']}")
    
    # 执行修正
    corrected_state, corrected_gyro_bias, corrected_accel_bias = eskf.correct_nominal_state(
        nominal_state, gyro_bias, accel_bias
    )
    
    print(f"   修正后位置：{corrected_state['position']}")
    print(f"   修正后速度：{corrected_state['velocity']}")
    print(f"   修正后姿态：{corrected_state['attitude']}")
    print(f"   修正后陀螺零偏：{corrected_gyro_bias}")
    print(f"   修正后加计零偏：{corrected_accel_bias}")
    
    # 验证误差状态已重置
    error_state_after = eskf.get_error_state()
    print(f"\n2. 验证误差状态重置：")
    print(f"   误差状态范数：{np.linalg.norm(error_state_after):.9f}")
    
    if np.linalg.norm(error_state_after) < 1e-10:
        print("   ✓ 误差状态已重置为零")
    else:
        print("   ✗ 警告：误差状态未正确重置")
    
    # 验证四元数归一化
    q_norm = np.linalg.norm(corrected_state['attitude'])
    print(f"\n3. 验证四元数归一化：")
    print(f"   四元数范数：{q_norm:.9f}")
    
    if abs(q_norm - 1.0) < 1e-6:
        print("   ✓ 四元数已正确归一化")
    else:
        print("   ✗ 警告：四元数未正确归一化")
    
    print("\n✓ ESKF状态修正测试通过")


def main():
    """主测试函数"""
    print("\n" + "=" * 60)
    print("ESKF误差状态模型测试")
    print("第四步：误差状态ESKF模型建立（状态与误差方程）")
    print("=" * 60)
    
    try:
        # 测试误差动力学模型
        test_error_dynamics()
        
        # 测试ESKF预测步骤
        test_eskf_predict()
        
        # 测试ESKF更新步骤
        test_eskf_update()
        
        # 测试ESKF状态修正
        test_eskf_correction()
        
        print("\n" + "=" * 60)
        print("✓ 所有测试通过！")
        print("=" * 60)
        
    except Exception as e:
        print("\n" + "=" * 60)
        print(f"✗ 测试失败：{e}")
        print("=" * 60)
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
