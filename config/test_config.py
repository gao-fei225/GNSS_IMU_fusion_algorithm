"""
配置管理器测试脚本
验证ConfigManager的功能
"""

import sys
from pathlib import Path

# 添加项目根目录到路径
sys.path.append(str(Path(__file__).parent.parent))

from config import ConfigManager


def test_config_manager():
    """
    测试配置管理器的各项功能
    """
    print("=" * 60)
    print("配置管理器功能测试")
    print("=" * 60)
    
    # 测试1：加载默认配置
    print("\n[测试1] 加载默认配置...")
    try:
        config = ConfigManager()
        print("✓ 默认配置加载成功")
    except Exception as e:
        print(f"✗ 默认配置加载失败: {e}")
        return
    
    # 测试2：验证配置
    print("\n[测试2] 验证配置完整性...")
    is_valid, errors = config.validate_config()
    if is_valid:
        print("✓ 配置验证通过")
    else:
        print("✗ 配置验证失败:")
        for error in errors:
            print(f"  - {error}")
    
    # 测试3：获取数据路径
    print("\n[测试3] 获取数据路径...")
    data_paths = config.get_data_paths()
    print(f"  IMU数据路径: {data_paths.get('imu_data')}")
    print(f"  GNSS导航解路径: {data_paths.get('gnss_nav')}")
    print(f"  输出目录: {data_paths.get('output_dir')}")
    print("✓ 数据路径获取成功")
    
    # 测试4：获取IMU参数
    print("\n[测试4] 获取IMU参数...")
    imu_params = config.get_imu_params()
    print(f"  采样频率: {imu_params.get('sample_rate')} Hz")
    print(f"  陀螺噪声: {imu_params.get('gyro_noise')} rad/s/√Hz")
    print("✓ IMU参数获取成功")
    
    # 测试5：获取GNSS参数
    print("\n[测试5] 获取GNSS参数...")
    gnss_params = config.get_gnss_params()
    print(f"  输出频率: {gnss_params.get('output_rate')} Hz")
    print(f"  位置噪声: {gnss_params.get('position_noise')} m")
    print("✓ GNSS参数获取成功")
    
    # 测试6：获取ESKF参数
    print("\n[测试6] 获取ESKF参数...")
    eskf_params = config.get_eskf_params()
    print(f"  状态维数: {eskf_params.get('state_dim')}")
    initial_cov = eskf_params.get('initial_cov', {})
    print(f"  初始姿态协方差: {initial_cov.get('attitude')} rad²")
    print("✓ ESKF参数获取成功")
    
    # 测试7：获取实验设置
    print("\n[测试7] 获取实验设置...")
    exp_settings = config.get_experiment_settings()
    print(f"  启用松耦合: {exp_settings.get('enable_loose_coupling')}")
    print(f"  启用紧耦合: {exp_settings.get('enable_tight_coupling')}")
    print(f"  GNSS遮挡时间段: {exp_settings.get('gnss_outages')}")
    print("✓ 实验设置获取成功")
    
    # 测试8：使用嵌套键获取值
    print("\n[测试8] 使用嵌套键获取值...")
    sample_rate = config.get_value('imu_params.sample_rate')
    attitude_cov = config.get_value('eskf_params.initial_cov.attitude')
    print(f"  IMU采样频率: {sample_rate}")
    print(f"  初始姿态协方差: {attitude_cov}")
    print("✓ 嵌套键获取成功")
    
    # 测试9：打印完整配置
    print("\n[测试9] 打印完整配置信息...")
    config.print_config()
    
    # 测试10：保存配置
    print("\n[测试10] 保存配置到文件...")
    try:
        output_path = Path(__file__).parent / 'test_output_config.yaml'
        config.save_config(output_path)
        print(f"✓ 配置已保存到: {output_path}")
        
        # 清理测试文件
        if output_path.exists():
            output_path.unlink()
            print("✓ 测试文件已清理")
    except Exception as e:
        print(f"✗ 配置保存失败: {e}")
    
    print("\n" + "=" * 60)
    print("所有测试完成！")
    print("=" * 60)


if __name__ == "__main__":
    test_config_manager()
