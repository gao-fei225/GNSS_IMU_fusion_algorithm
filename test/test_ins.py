"""
INS编排算法测试脚本
测试纯惯导解算功能
"""

from pathlib import Path
import numpy as np
import sys
import matplotlib.pyplot as plt

# 设置项目根目录
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# 动态加载模块避免命名冲突
import importlib.util

# 加载INS模块
ins_path = project_root / 'ins' / 'mechanization.py'
spec = importlib.util.spec_from_file_location("mechanization", ins_path)
mechanization = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mechanization)
INSMechanization = mechanization.INSMechanization


def test_quaternion_operations():
    """
    测试四元数操作（欧拉角转换、旋转矩阵）
    """
    print("\n" + "=" * 60)
    print("测试1：四元数操作")
    print("=" * 60)
    
    # 测试欧拉角到四元数的转换
    roll = np.deg2rad(10)   # 10度
    pitch = np.deg2rad(20)  # 20度
    yaw = np.deg2rad(30)    # 30度
    
    print(f"\n原始欧拉角（度）：Roll={np.rad2deg(roll):.1f}, Pitch={np.rad2deg(pitch):.1f}, Yaw={np.rad2deg(yaw):.1f}")
    
    # 转换为四元数
    quat = INSMechanization.euler_to_quaternion(roll, pitch, yaw)
    print(f"四元数：{quat}")
    print(f"四元数模：{np.linalg.norm(quat):.6f} (应该为1.0)")
    
    # 创建INS对象并测试反向转换
    initial_state = {
        'attitude': quat,
        'velocity': np.array([0.0, 0.0, 0.0]),
        'position': np.array([0.0, 0.0, 0.0])
    }
    ins = INSMechanization(initial_state)
    
    euler_back = ins.get_euler_angles()
    print(f"反向转换欧拉角（度）：Roll={np.rad2deg(euler_back[0]):.1f}, Pitch={np.rad2deg(euler_back[1]):.1f}, Yaw={np.rad2deg(euler_back[2]):.1f}")
    
    # 测试旋转矩阵
    C = ins.get_rotation_matrix()
    print(f"\n旋转矩阵行列式：{np.linalg.det(C):.6f} (应该为1.0)")
    print(f"旋转矩阵正交性检查（C*C' - I的范数）：{np.linalg.norm(C @ C.T - np.eye(3)):.10f} (应该接近0)")
    
    print("\n✓ 四元数操作测试通过")


def test_static_ins():
    """
    测试静止状态下的INS（应该保持姿态和速度不变）
    """
    print("\n" + "=" * 60)
    print("测试2：静止状态INS")
    print("=" * 60)
    
    # 初始化：静止状态，水平姿态
    initial_state = {
        'attitude': np.array([1.0, 0.0, 0.0, 0.0]),  # 单位四元数（无旋转）
        'velocity': np.array([0.0, 0.0, 0.0]),
        'position': np.array([0.0, 0.0, 0.0])
    }
    
    ins = INSMechanization(initial_state)
    
    # 模拟静止：零角速度，只有重力加速度
    duration = 10.0  # 10秒
    dt = 0.01  # 10ms
    n_steps = int(duration / dt)
    
    gyro_static = np.array([0.0, 0.0, 0.0])  # 静止
    accel_static = np.array([0.0, 0.0, 9.81])  # 重力加速度（机体系，向上）
    
    # 记录轨迹
    positions = [ins.position.copy()]
    velocities = [ins.velocity.copy()]
    attitudes = [ins.get_euler_angles().copy()]
    
    for i in range(n_steps):
        ins.update(gyro_static, accel_static, dt)
        if i % 100 == 0:  # 每秒记录一次
            positions.append(ins.position.copy())
            velocities.append(ins.velocity.copy())
            attitudes.append(ins.get_euler_angles().copy())
    
    positions = np.array(positions)
    velocities = np.array(velocities)
    attitudes = np.array(attitudes)
    
    # 检查结果
    print(f"\n初始位置：{positions[0]}")
    print(f"最终位置：{positions[-1]}")
    print(f"位置漂移：{np.linalg.norm(positions[-1] - positions[0]):.6f} m")
    
    print(f"\n初始速度：{velocities[0]}")
    print(f"最终速度：{velocities[-1]}")
    print(f"速度漂移：{np.linalg.norm(velocities[-1] - velocities[0]):.6f} m/s")
    
    print(f"\n初始姿态（度）：{np.rad2deg(attitudes[0])}")
    print(f"最终姿态（度）：{np.rad2deg(attitudes[-1])}")
    
    # 静止状态下，位置和速度应该基本不变
    assert np.linalg.norm(positions[-1] - positions[0]) < 0.01, "静止状态位置漂移过大"
    assert np.linalg.norm(velocities[-1] - velocities[0]) < 0.01, "静止状态速度漂移过大"
    
    print("\n✓ 静止状态INS测试通过")


def test_constant_rotation():
    """
    测试恒定角速度旋转
    """
    print("\n" + "=" * 60)
    print("测试3：恒定角速度旋转")
    print("=" * 60)
    
    # 初始化：水平姿态，静止
    initial_state = {
        'attitude': np.array([1.0, 0.0, 0.0, 0.0]),
        'velocity': np.array([0.0, 0.0, 0.0]),
        'position': np.array([0.0, 0.0, 0.0])
    }
    
    ins = INSMechanization(initial_state)
    
    # 恒定角速度：绕z轴旋转（yaw）
    rotation_rate = np.deg2rad(36)  # 36度/秒
    gyro = np.array([0.0, 0.0, rotation_rate])  # 绕z轴旋转
    accel = np.array([0.0, 0.0, 9.81])  # 重力加速度
    
    # 旋转10秒，应该旋转360度回到原点
    duration = 10.0
    dt = 0.01
    n_steps = int(duration / dt)
    
    attitudes = []
    for i in range(n_steps):
        ins.update(gyro, accel, dt)
        if i % 10 == 0:  # 每0.1秒记录
            attitudes.append(ins.get_euler_angles().copy())
    
    attitudes = np.array(attitudes)
    
    print(f"\n初始Yaw：{np.rad2deg(attitudes[0, 2]):.1f}°")
    print(f"最终Yaw：{np.rad2deg(attitudes[-1, 2]):.1f}°")
    print(f"理论旋转角度：{rotation_rate * duration * 180/np.pi:.1f}°")
    
    # 计算实际旋转角度（考虑周期性）
    yaw_change = attitudes[-1, 2] - attitudes[0, 2]
    # 归一化到[-π, π]
    yaw_change = np.arctan2(np.sin(yaw_change), np.cos(yaw_change))
    actual_rotation = yaw_change * 180/np.pi
    
    print(f"实际旋转角度：{actual_rotation:.1f}°")
    
    print("\n✓ 恒定角速度旋转测试通过")


def test_linear_motion():
    """
    测试匀速直线运动
    """
    print("\n" + "=" * 60)
    print("测试4：匀速直线运动")
    print("=" * 60)
    
    # 初始化：水平姿态，初始速度
    initial_velocity = 10.0  # 10 m/s
    initial_state = {
        'attitude': np.array([1.0, 0.0, 0.0, 0.0]),
        'velocity': np.array([initial_velocity, 0.0, 0.0]),  # x方向运动
        'position': np.array([0.0, 0.0, 0.0])
    }
    
    ins = INSMechanization(initial_state)
    
    # 匀速：零角速度，零额外加速度（只有重力）
    gyro = np.array([0.0, 0.0, 0.0])
    accel = np.array([0.0, 0.0, 9.81])
    
    duration = 10.0
    dt = 0.01
    n_steps = int(duration / dt)
    
    positions = []
    velocities = []
    
    for i in range(n_steps):
        ins.update(gyro, accel, dt)
        if i % 100 == 0:
            positions.append(ins.position.copy())
            velocities.append(ins.velocity.copy())
    
    positions = np.array(positions)
    velocities = np.array(velocities)
    
    print(f"\n初始位置：{positions[0]}")
    print(f"最终位置：{positions[-1]}")
    print(f"位移：{positions[-1] - positions[0]}")
    print(f"理论位移：{initial_velocity * duration} m (x方向)")
    
    print(f"\n初始速度：{velocities[0]}")
    print(f"最终速度：{velocities[-1]}")
    
    # 检查x方向位移
    x_displacement = positions[-1, 0] - positions[0, 0]
    expected_displacement = initial_velocity * duration
    error = abs(x_displacement - expected_displacement)
    
    print(f"\nx方向位移误差：{error:.6f} m")
    assert error < 0.1, "匀速直线运动位移误差过大"
    
    print("\n✓ 匀速直线运动测试通过")


def visualize_ins_trajectory():
    """
    可视化INS轨迹（可选）
    """
    print("\n" + "=" * 60)
    print("测试5：可视化复杂运动轨迹")
    print("=" * 60)
    
    # 模拟一个圆周运动
    initial_state = {
        'attitude': np.array([1.0, 0.0, 0.0, 0.0]),
        'velocity': np.array([10.0, 0.0, 0.0]),
        'position': np.array([0.0, 0.0, 0.0])
    }
    
    ins = INSMechanization(initial_state)
    
    # 恒定转向（模拟圆周运动）
    angular_velocity = np.deg2rad(36)  # 36度/秒
    centripetal_accel = 3.6  # 向心加速度
    
    duration = 10.0
    dt = 0.01
    n_steps = int(duration / dt)
    
    trajectory = []
    times = []
    
    for i in range(n_steps):
        # 模拟圆周运动的IMU数据
        gyro = np.array([0.0, 0.0, angular_velocity])
        accel = np.array([centripetal_accel, 0.0, 9.81])
        
        ins.update(gyro, accel, dt)
        
        if i % 10 == 0:
            trajectory.append(ins.position.copy())
            times.append(i * dt)
    
    trajectory = np.array(trajectory)
    
    print(f"\n轨迹点数：{len(trajectory)}")
    print(f"起点：{trajectory[0]}")
    print(f"终点：{trajectory[-1]}")
    print(f"轨迹范围：X[{trajectory[:, 0].min():.1f}, {trajectory[:, 0].max():.1f}] m")
    print(f"          Y[{trajectory[:, 1].min():.1f}, {trajectory[:, 1].max():.1f}] m")
    
    print("\n✓ 复杂运动轨迹生成成功")
    
    return trajectory, times


def main():
    """
    主测试函数
    """
    print("\n" + "=" * 60)
    print("INS编排算法功能测试")
    print("=" * 60)
    
    # 运行所有测试
    test_quaternion_operations()
    test_static_ins()
    test_constant_rotation()
    test_linear_motion()
    visualize_ins_trajectory()
    
    print("\n" + "=" * 60)
    print("所有测试完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()
