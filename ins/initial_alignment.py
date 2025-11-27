"""
INS初始对齐模块
提供静态对齐和GNSS辅助对齐功能
"""

import numpy as np


def static_alignment(imu_data, duration=10.0, coordinate_frame='ENU'):
    """
    静态对齐（假设载体静止）
    
    利用静止时的IMU数据确定初始姿态
    - Roll, Pitch: 通过平均加速度计读数（重力方向）确定
    - Yaw: 无法确定（需要磁力计或GNSS速度）
    
    Args:
        imu_data: IMU数据字典，包含timestamp, gyro, accel
        duration: 对齐时长（秒），默认10秒
        coordinate_frame: 坐标系类型 ('ENU', 'NED', 'FRD')
    
    Returns:
        dict: 初始状态，包含attitude（四元数）, velocity, position
    """
    timestamps = imu_data['timestamp']
    accels = imu_data['accel']
    
    # 选择前N秒的数据
    mask = timestamps <= (timestamps[0] + duration)
    accel_static = accels[mask]
    
    # 平均加速度（消除噪声）
    accel_mean = accel_static.mean(axis=0)
    
    print(f"\n静态对齐（{duration}秒）:")
    print(f"  平均加速度: {accel_mean}")
    print(f"  模值: {np.linalg.norm(accel_mean):.2f} m/s²")
    
    # 根据重力方向计算Roll和Pitch
    # 加速度计测量的是比力 f = a - g，静止时 a=0，所以 f = -g
    # 即加速度计读数指向重力的反方向（向上）
    
    if coordinate_frame == 'ENU':
        # ENU: Z向上，静止时加速度计应读 [0, 0, 9.81]
        # 归一化
        g_body = -accel_mean / np.linalg.norm(accel_mean)  # 重力在机体系的方向
        # g_body应该是[0, 0, -1]（向下）
        
        # Roll: 绕X轴旋转（东轴）
        roll = np.arctan2(g_body[1], g_body[2])
        
        # Pitch: 绕Y轴旋转（北轴）
        pitch = np.arctan2(-g_body[0], np.sqrt(g_body[1]**2 + g_body[2]**2))
        
    elif coordinate_frame in ['NED', 'FRD']:
        # NED/FRD: Z向下，静止时加速度计应读 [0, 0, -9.81]
        # 归一化
        g_body = -accel_mean / np.linalg.norm(accel_mean)  # 重力在机体系的方向
        # g_body应该是[0, 0, 1]（向下）
        
        # Roll: 绕X轴旋转
        roll = np.arctan2(-g_body[1], g_body[2])
        
        # Pitch: 绕Y轴旋转  
        pitch = np.arctan2(g_body[0], np.sqrt(g_body[1]**2 + g_body[2]**2))
    
    # Yaw无法确定，设为0
    yaw = 0.0
    
    print(f"  估计姿态: Roll={np.rad2deg(roll):.2f}°, Pitch={np.rad2deg(pitch):.2f}°, Yaw={np.rad2deg(yaw):.2f}°")
    print(f"  ⚠️  注意：Yaw角无法通过静态对齐确定，设为0°")
    
    # 转换为四元数
    from .mechanization import INSMechanization
    attitude = INSMechanization.euler_to_quaternion(roll, pitch, yaw)
    
    return {
        'attitude': attitude,
        'velocity': np.array([0.0, 0.0, 0.0]),  # 静止
        'position': np.array([0.0, 0.0, 0.0])   # 原点
    }


def gnss_aided_alignment(imu_data, gnss_data, duration=10.0, coordinate_frame='ENU'):
    """
    GNSS辅助对齐
    
    利用GNSS速度辅助确定Yaw角
    
    Args:
        imu_data: IMU数据字典
        gnss_data: GNSS数据字典，包含position, velocity
        duration: 对齐时长
        coordinate_frame: 坐标系类型
    
    Returns:
        dict: 初始状态
    """
    # 先进行静态对齐得到Roll和Pitch
    alignment = static_alignment(imu_data, duration, coordinate_frame)
    
    # 使用GNSS速度确定Yaw
    # 假设gnss_data['velocity']第一个有效值
    gnss_vel = gnss_data['velocity'][0] if hasattr(gnss_data['velocity'], '__len__') else gnss_data['velocity']
    
    # 计算Yaw（航向角）
    if coordinate_frame == 'ENU':
        # ENU: Yaw从东向北为正（逆时针）
        yaw = np.arctan2(gnss_vel[1], gnss_vel[0])  # atan2(北, 东)
    elif coordinate_frame == 'NED':
        # NED: Yaw从北向东为正（顺时针）
        yaw = np.arctan2(gnss_vel[1], gnss_vel[0])  # atan2(东, 北)
    elif coordinate_frame == 'FRD':
        # FRD: Yaw从前向右为正
        yaw = np.arctan2(gnss_vel[1], gnss_vel[0])
    
    # 重新计算四元数
    roll = np.arcsin(2 * (alignment['attitude'][0] * alignment['attitude'][1] + 
                          alignment['attitude'][2] * alignment['attitude'][3]))
    pitch = np.arctan2(2 * (alignment['attitude'][0] * alignment['attitude'][2] - 
                            alignment['attitude'][3] * alignment['attitude'][1]),
                      1 - 2 * (alignment['attitude'][1]**2 + alignment['attitude'][2]**2))
    
    from .mechanization import INSMechanization
    attitude = INSMechanization.euler_to_quaternion(roll, pitch, yaw)
    
    print(f"\nGNSS辅助对齐:")
    print(f"  GNSS速度: {gnss_vel}")
    print(f"  估计Yaw: {np.rad2deg(yaw):.2f}°")
    
    return {
        'attitude': attitude,
        'velocity': gnss_vel,
        'position': gnss_data['position'][0] if hasattr(gnss_data['position'], '__len__') else gnss_data['position']
    }


def load_initial_state_from_truth(truth_file, navigation_frame='ENU'):
    """
    从真值文件加载初始状态（仅用于测试验证）
    
    Args:
        truth_file: 真值文件路径
        navigation_frame: 导航坐标系类型 ('ENU' 或 'NED')
    
    Returns:
        dict: 初始状态
    """
    data = np.loadtxt(truth_file)
    
    if data.shape[1] == 10:
        # gnss-ins-sim格式: timestamp pos_x pos_y pos_z vel_x vel_y vel_z roll pitch yaw
        position = data[0, 1:4]
        velocity = data[0, 4:7]
        roll = data[0, 7]
        pitch = data[0, 8]
        yaw = data[0, 9]
    else:
        # Awesome GINS格式: GPS周 GPS周秒 纬度 经度 高度 速度N 速度E 速度D Roll Pitch Yaw
        # 真值速度定义：北-东-下 (NED坐标系)
        position = np.array([0.0, 0.0, 0.0])  # 以第一点为原点
        
        vel_n, vel_e, vel_d = data[0, 5:8]  # 北-东-下
        
        if navigation_frame == 'NED':
            # NED导航系：直接使用真值速度
            velocity = np.array([vel_n, vel_e, vel_d])
        elif navigation_frame == 'ENU':
            # ENU导航系：需要转换
            velocity = np.array([vel_e, vel_n, -vel_d])
        else:
            raise ValueError(f"不支持的导航坐标系: {navigation_frame}")
        
        roll = np.deg2rad(data[0, 8])
        pitch = np.deg2rad(data[0, 9])
        yaw = np.deg2rad(data[0, 10])
    
    from .mechanization import INSMechanization
    attitude = INSMechanization.euler_to_quaternion(roll, pitch, yaw)
    
    print(f"\n从真值加载初始状态 (导航系: {navigation_frame}):")
    print(f"  位置: {position}")
    print(f"  速度: {velocity}")
    if navigation_frame == 'NED':
        print(f"        [北={velocity[0]:.4f}, 东={velocity[1]:.4f}, 下={velocity[2]:.4f}] m/s")
    elif navigation_frame == 'ENU':
        print(f"        [东={velocity[0]:.4f}, 北={velocity[1]:.4f}, 天={velocity[2]:.4f}] m/s")
    print(f"  姿态: Roll={np.rad2deg(roll):.2f}°, Pitch={np.rad2deg(pitch):.2f}°, Yaw={np.rad2deg(yaw):.2f}°")
    
    return {
        'attitude': attitude,
        'velocity': velocity,
        'position': position
    }
