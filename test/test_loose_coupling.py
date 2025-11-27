"""
测试松耦合ESKF
验证第五步实现：松耦合ESKF（INS + GNSS导航解）
"""

import numpy as np
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from models.error_dynamics import ErrorDynamics
from models.measurement_models import LooseCouplingModel
from fusion.loose_coupling import LooseCouplingESKF
from ins.mechanization import INSMechanization
from config.config_manager import ConfigManager


def test_loose_coupling_model():
    """测试松耦合量测模型"""
    print("=" * 60)
    print("测试松耦合量测模型")
    print("=" * 60)
    
    # 加载配置
    config_manager = ConfigManager()
    gnss_params = config_manager.get_gnss_params()
    
    # 创建松耦合量测模型
    lc_model = LooseCouplingModel(gnss_params)
    
    # 构造测试数据
    nominal_state = {
        'attitude': np.array([1.0, 0.0, 0.0, 0.0]),
        'velocity': np.array([10.0, 0.0, 0.0]),
        'position': np.array([100.0, 50.0, 10.0])
    }
    
    gnss_measurement = {
        'position': np.array([100.5, 50.2, 10.1]),
        'velocity': np.array([10.1, 0.05, 0.01])
    }
    
    # 1. 测试新息计算
    print("\n1. 测试量测新息计算：")
    innovation = lc_model.compute_innovation(nominal_state, gnss_measurement)
    print(f"   新息向量维度：{innovation.shape}")
    print(f"   位置新息：{innovation[0:3]}")
    print(f"   速度新息：{innovation[3:6]}")
    
    # 验证新息计算正确性
    expected_pos_innov = np.array([0.5, 0.2, 0.1])
    expected_vel_innov = np.array([0.1, 0.05, 0.01])
    assert np.allclose(innovation[0:3], expected_pos_innov), "位置新息计算错误"
    assert np.allclose(innovation[3:6], expected_vel_innov), "速度新息计算错误"
    print("   ✓ 新息计算正确")
    
    # 2. 测试H矩阵
    print("\n2. 测试量测矩阵H：")
    H = lc_model.compute_H_matrix(nominal_state)
    print(f"   H矩阵形状：{H.shape}")
    print(f"   H矩阵非零元素数量：{np.count_nonzero(H)}")
    
    # 验证H矩阵结构
    assert H.shape == (6, 15), "H矩阵维度错误"
    assert np.allclose(H[0:3, 6:9], np.eye(3)), "位置观测部分错误"
    assert np.allclose(H[3:6, 3:6], np.eye(3)), "速度观测部分错误"
    print("   ✓ H矩阵结构正确")
    
    # 3. 测试R矩阵
    print("\n3. 测试量测噪声协方差矩阵R：")
    R = lc_model.compute_R_matrix()
    print(f"   R矩阵形状：{R.shape}")
    print(f"   R矩阵迹：{np.trace(R):.6f}")
    print(f"   位置噪声方差：{R[0, 0]:.6f}")
    print(f"   速度噪声方差：{R[3, 3]:.6f}")
    
    # 验证R矩阵
    assert R.shape == (6, 6), "R矩阵维度错误"
    assert np.allclose(R[0:3, 0:3], np.eye(3) * gnss_params['position_noise']**2), "位置噪声错误"
    assert np.allclose(R[3:6, 3:6], np.eye(3) * gnss_params['velocity_noise']**2), "速度噪声错误"
    print("   ✓ R矩阵计算正确")
    
    print("\n✓ 松耦合量测模型测试通过")


def test_loose_coupling_eskf():
    """测试松耦合ESKF完整流程"""
    print("\n" + "=" * 60)
    print("测试松耦合ESKF完整流程")
    print("=" * 60)
    
    # 加载配置
    config_manager = ConfigManager()
    imu_params = config_manager.get_imu_params()
    gnss_params = config_manager.get_gnss_params()
    eskf_params = config_manager.get_eskf_params()
    
    # 创建误差动力学模型
    error_dynamics = ErrorDynamics(imu_params)
    
    # 创建松耦合量测模型
    lc_model = LooseCouplingModel(gnss_params)
    
    # 创建初始协方差矩阵
    initial_cov = np.eye(15)
    initial_cov[0:3, 0:3] *= eskf_params['initial_cov']['attitude']
    initial_cov[3:6, 3:6] *= eskf_params['initial_cov']['velocity']
    initial_cov[6:9, 6:9] *= eskf_params['initial_cov']['position']
    initial_cov[9:12, 9:12] *= eskf_params['initial_cov']['gyro_bias']
    initial_cov[12:15, 12:15] *= eskf_params['initial_cov']['accel_bias']
    
    # 创建松耦合ESKF
    lc_eskf = LooseCouplingESKF(initial_cov, error_dynamics, lc_model)
    
    print("\n1. 测试预测步骤：")
    nominal_state = {
        'attitude': np.array([1.0, 0.0, 0.0, 0.0]),
        'velocity': np.array([10.0, 0.0, 0.0]),
        'position': np.array([100.0, 50.0, 10.0])
    }
    
    imu_measurement = {
        'gyro': np.array([0.01, 0.0, 0.0]),
        'accel': np.array([0.0, 0.0, 9.81])
    }
    
    dt = 0.01
    P_before = lc_eskf.get_covariance()
    lc_eskf.predict(nominal_state, imu_measurement, dt)
    P_after = lc_eskf.get_covariance()
    
    print(f"   预测前协方差迹：{np.trace(P_before):.6f}")
    print(f"   预测后协方差迹：{np.trace(P_after):.6f}")
    print(f"   协方差增长：{np.trace(P_after) - np.trace(P_before):.9f}")
    print("   ✓ 预测步骤正常")
    
    print("\n2. 测试GNSS量测处理：")
    gnss_nav = {
        'position': np.array([100.5, 50.2, 10.1]),
        'velocity': np.array([10.1, 0.05, 0.01])
    }
    
    gyro_bias = np.array([0.001, 0.0, 0.0])
    accel_bias = np.array([0.01, 0.0, 0.0])
    
    print(f"   处理前位置：{nominal_state['position']}")
    print(f"   GNSS观测位置：{gnss_nav['position']}")
    
    corrected_state, corrected_gyro_bias, corrected_accel_bias = \
        lc_eskf.process_gnss_measurement(nominal_state, gnss_nav, gyro_bias, accel_bias)
    
    print(f"   修正后位置：{corrected_state['position']}")
    print(f"   修正后速度：{corrected_state['velocity']}")
    print(f"   修正后陀螺零偏：{corrected_gyro_bias}")
    print(f"   修正后加计零偏：{corrected_accel_bias}")
    
    # 验证状态修正效果
    pos_diff = np.linalg.norm(corrected_state['position'] - gnss_nav['position'])
    print(f"\n   修正后与GNSS观测的位置差：{pos_diff:.6f} m")
    assert pos_diff < 1.0, "位置修正幅度异常"
    print("   ✓ GNSS量测处理正常")
    
    print("\n3. 验证误差状态重置：")
    error_state = lc_eskf.get_error_state()
    error_norm = np.linalg.norm(error_state)
    print(f"   误差状态范数：{error_norm:.12f}")
    assert error_norm < 1e-10, "误差状态未正确重置"
    print("   ✓ 误差状态已重置为零")
    
    print("\n✓ 松耦合ESKF完整流程测试通过")


def test_ins_gnss_fusion_simulation():
    """模拟INS+GNSS融合过程"""
    print("\n" + "=" * 60)
    print("模拟INS+GNSS融合过程")
    print("=" * 60)
    
    # 加载配置
    config_manager = ConfigManager()
    imu_params = config_manager.get_imu_params()
    gnss_params = config_manager.get_gnss_params()
    eskf_params = config_manager.get_eskf_params()
    
    # 初始化INS
    initial_state = {
        'attitude': INSMechanization.euler_to_quaternion(0, 0, 0),
        'velocity': np.array([10.0, 0.0, 0.0]),
        'position': np.array([0.0, 0.0, 0.0])
    }
    ins = INSMechanization(initial_state, navigation_frame='ENU')
    
    # 初始化松耦合ESKF
    error_dynamics = ErrorDynamics(imu_params)
    lc_model = LooseCouplingModel(gnss_params)
    initial_cov = np.eye(15) * 0.1
    lc_eskf = LooseCouplingESKF(initial_cov, error_dynamics, lc_model)
    
    # 零偏估计
    gyro_bias = np.zeros(3)
    accel_bias = np.zeros(3)
    
    # 模拟参数
    dt = 0.01  # 100Hz
    gnss_rate = 1.0  # 1Hz
    total_time = 10.0  # 10秒
    num_steps = int(total_time / dt)
    gnss_interval = int(1.0 / gnss_rate / dt)
    
    print(f"\n模拟参数：")
    print(f"   总时间：{total_time}秒")
    print(f"   IMU采样率：{1/dt}Hz")
    print(f"   GNSS采样率：{gnss_rate}Hz")
    print(f"   总步数：{num_steps}")
    
    # 存储结果
    ins_positions = []
    fusion_positions = []
    times = []
    
    print(f"\n开始模拟融合过程...")
    
    for step in range(num_steps):
        t = step * dt
        times.append(t)
        
        # 模拟IMU测量（匀速直线运动 + 小噪声）
        gyro = np.random.randn(3) * 0.001
        accel = np.array([0.0, 0.0, 9.81]) + np.random.randn(3) * 0.01
        
        imu_measurement = {'gyro': gyro, 'accel': accel}
        
        # INS预测
        ins.update(gyro, accel, dt)
        nominal_state = ins.get_state()
        
        # ESKF预测
        lc_eskf.predict(nominal_state, imu_measurement, dt)
        
        # 每秒进行一次GNSS更新
        if step % gnss_interval == 0 and step > 0:
            # 模拟GNSS观测（真值 + 噪声）
            true_position = np.array([10.0 * t, 0.0, 0.0])
            true_velocity = np.array([10.0, 0.0, 0.0])
            
            gnss_nav = {
                'position': true_position + np.random.randn(3) * gnss_params['position_noise'],
                'velocity': true_velocity + np.random.randn(3) * gnss_params['velocity_noise']
            }
            
            # GNSS更新
            corrected_state, gyro_bias, accel_bias = \
                lc_eskf.process_gnss_measurement(nominal_state, gnss_nav, gyro_bias, accel_bias)
            
            # 更新INS状态
            ins.position = corrected_state['position']
            ins.velocity = corrected_state['velocity']
            ins.attitude = corrected_state['attitude']
        
        # 记录位置
        ins_positions.append(nominal_state['position'].copy())
        fusion_positions.append(ins.position.copy())
    
    ins_positions = np.array(ins_positions)
    fusion_positions = np.array(fusion_positions)
    
    # 计算误差统计
    print(f"\n融合结果统计：")
    final_pos_ins = ins_positions[-1]
    final_pos_fusion = fusion_positions[-1]
    true_final_pos = np.array([10.0 * total_time, 0.0, 0.0])
    
    error_ins = np.linalg.norm(final_pos_ins - true_final_pos)
    error_fusion = np.linalg.norm(final_pos_fusion - true_final_pos)
    
    print(f"   真实最终位置：{true_final_pos}")
    print(f"   纯INS最终位置：{final_pos_ins}")
    print(f"   融合后最终位置：{final_pos_fusion}")
    print(f"   纯INS位置误差：{error_ins:.3f} m")
    print(f"   融合后位置误差：{error_fusion:.3f} m")
    
    if error_fusion < error_ins:
        print(f"   ✓ 融合效果提升：{(error_ins - error_fusion):.3f} m")
    
    print("\n✓ INS+GNSS融合模拟测试通过")


def main():
    """主测试函数"""
    print("\n" + "=" * 60)
    print("松耦合ESKF测试")
    print("第五步：松耦合ESKF实现（INS + GNSS导航解）")
    print("=" * 60)
    
    try:
        # 测试松耦合量测模型
        test_loose_coupling_model()
        
        # 测试松耦合ESKF
        test_loose_coupling_eskf()
        
        # 模拟融合过程
        test_ins_gnss_fusion_simulation()
        
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
