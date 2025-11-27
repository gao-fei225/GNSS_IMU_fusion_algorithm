"""
松耦合ESKF使用示例
展示如何使用松耦合ESKF进行INS/GNSS融合
"""

import numpy as np
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.config_manager import ConfigManager
from models.error_dynamics import ErrorDynamics
from models.measurement_models import LooseCouplingModel
from fusion.loose_coupling import LooseCouplingESKF
from ins.mechanization import INSMechanization


def main():
    """松耦合ESKF使用示例"""
    print("=" * 60)
    print("松耦合ESKF使用示例")
    print("=" * 60)
    
    # ========== 1. 加载配置 ==========
    print("\n[1] 加载配置...")
    config_manager = ConfigManager()
    imu_params = config_manager.get_imu_params()
    gnss_params = config_manager.get_gnss_params()
    eskf_params = config_manager.get_eskf_params()
    print("   ✓ 配置加载完成")
    
    # ========== 2. 初始化INS ==========
    print("\n[2] 初始化INS机械编排...")
    initial_state = {
        'attitude': INSMechanization.euler_to_quaternion(0, 0, 0),  # 初始姿态：水平
        'velocity': np.array([0.0, 0.0, 0.0]),                      # 初始速度：静止
        'position': np.array([0.0, 0.0, 0.0])                       # 初始位置：原点
    }
    ins = INSMechanization(initial_state, navigation_frame='ENU')
    print(f"   初始位置：{initial_state['position']}")
    print(f"   初始速度：{initial_state['velocity']}")
    print("   ✓ INS初始化完成")
    
    # ========== 3. 初始化松耦合ESKF ==========
    print("\n[3] 初始化松耦合ESKF...")
    
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
    print(f"   初始协方差迹：{np.trace(initial_cov):.6f}")
    print("   ✓ ESKF初始化完成")
    
    # 初始零偏估计
    gyro_bias = np.zeros(3)
    accel_bias = np.zeros(3)
    
    # ========== 4. 模拟融合过程 ==========
    print("\n[4] 开始模拟融合过程...")
    
    # 模拟参数
    dt = 0.01          # IMU采样间隔：10ms (100Hz)
    gnss_dt = 1.0      # GNSS更新间隔：1s (1Hz)
    total_time = 5.0   # 总时间：5秒
    
    # 模拟运动：匀加速直线运动
    # 初始速度0，加速度1 m/s²，沿X轴方向
    acceleration = 1.0
    
    time = 0.0
    gnss_time = 0.0
    step = 0
    
    print(f"   模拟时间：{total_time}秒")
    print(f"   IMU频率：{1/dt}Hz")
    print(f"   GNSS频率：{1/gnss_dt}Hz")
    print(f"   运动模式：匀加速直线运动（a={acceleration} m/s²）")
    
    while time < total_time:
        step += 1
        
        # 模拟IMU测量（理想值 + 小噪声）
        # 角速度：0（直线运动）
        gyro = np.random.randn(3) * imu_params['gyro_noise']
        
        # 加速度：[a, 0, g]（X方向加速 + 重力补偿）
        accel = np.array([acceleration, 0.0, 9.81]) + \
                np.random.randn(3) * imu_params['accel_noise']
        
        # INS预测
        ins.update(gyro - gyro_bias, accel - accel_bias, dt)
        nominal_state = ins.get_state()
        
        # ESKF预测
        imu_measurement = {'gyro': gyro, 'accel': accel}
        lc_eskf.predict(nominal_state, imu_measurement, dt)
        
        # GNSS更新（每秒一次）
        if time - gnss_time >= gnss_dt:
            gnss_time = time
            
            # 计算真实位置和速度（匀加速运动）
            true_velocity = np.array([acceleration * time, 0.0, 0.0])
            true_position = np.array([0.5 * acceleration * time**2, 0.0, 0.0])
            
            # 模拟GNSS观测（真值 + 噪声）
            gnss_nav = {
                'position': true_position + np.random.randn(3) * gnss_params['position_noise'],
                'velocity': true_velocity + np.random.randn(3) * gnss_params['velocity_noise']
            }
            
            # GNSS更新
            corrected_state, gyro_bias, accel_bias = \
                lc_eskf.process_gnss_measurement(nominal_state, gnss_nav, 
                                                gyro_bias, accel_bias)
            
            # 更新INS状态
            ins.position = corrected_state['position']
            ins.velocity = corrected_state['velocity']
            ins.attitude = corrected_state['attitude']
            
            # 打印融合结果
            pos_error = np.linalg.norm(corrected_state['position'] - true_position)
            vel_error = np.linalg.norm(corrected_state['velocity'] - true_velocity)
            
            print(f"\n   时刻 {time:.1f}s：GNSS更新")
            print(f"     真实位置：{true_position}")
            print(f"     估计位置：{corrected_state['position']}")
            print(f"     位置误差：{pos_error:.4f} m")
            print(f"     速度误差：{vel_error:.4f} m/s")
        
        time += dt
    
    print("\n   ✓ 融合过程完成")
    
    # ========== 5. 最终结果 ==========
    print("\n[5] 最终结果：")
    final_state = ins.get_state()
    true_final_position = np.array([0.5 * acceleration * total_time**2, 0.0, 0.0])
    true_final_velocity = np.array([acceleration * total_time, 0.0, 0.0])
    
    final_pos_error = np.linalg.norm(final_state['position'] - true_final_position)
    final_vel_error = np.linalg.norm(final_state['velocity'] - true_final_velocity)
    
    print(f"   真实最终位置：{true_final_position}")
    print(f"   估计最终位置：{final_state['position']}")
    print(f"   最终位置误差：{final_pos_error:.4f} m")
    print(f"   最终速度误差：{final_vel_error:.4f} m/s")
    
    # 协方差分析
    final_cov = lc_eskf.get_covariance()
    pos_std = np.sqrt(np.diag(final_cov[6:9]))
    vel_std = np.sqrt(np.diag(final_cov[3:6]))
    
    print(f"\n   位置标准差（1σ）：{pos_std}")
    print(f"   速度标准差（1σ）：{vel_std}")
    
    print("\n" + "=" * 60)
    print("✓ 示例运行完成！")
    print("=" * 60)
    
    # ========== 使用提示 ==========
    print("\n使用提示：")
    print("1. 松耦合ESKF适用于GNSS信号良好的开阔环境")
    print("2. 量测更新频率通常为1-10Hz")
    print("3. IMU采样频率通常为100-200Hz")
    print("4. 需要定期进行GNSS更新以抑制INS漂移")
    print("5. 可以通过调整R矩阵来反映GNSS观测质量")


if __name__ == "__main__":
    main()
