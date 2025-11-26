# 纯INS惯导解算使用说明

## 功能说明

纯INS解算脚本可以读取IMU数据，进行惯导积分，输出位置、速度、姿态轨迹。

## 使用方法

### 方法1：命令行运行

```bash
python runners/run_pure_ins.py --imu data/your_imu_data.txt --output results/pure_ins
```

**参数说明：**
- `--imu`: IMU数据文件路径（必需）
- `--output`: 输出目录，保存结果和图表（可选，默认为`results/pure_ins`）
- `--ref`: 参考轨迹文件，用于误差评估（可选）

### 方法2：Python脚本调用

```python
from pathlib import Path
import sys

# 添加项目路径
project_root = Path('你的项目路径')
sys.path.insert(0, str(project_root))

# 导入并运行
from runners.run_pure_ins import run_pure_ins

trajectory = run_pure_ins(
    imu_file='data/your_imu_data.txt',
    output_dir='results/pure_ins'
)

# 访问结果
print(trajectory['position'])  # 位置轨迹
print(trajectory['velocity'])  # 速度轨迹
print(trajectory['attitude_euler'])  # 姿态（欧拉角）
```

## IMU数据格式

IMU数据文件应为文本格式（空格或逗号分隔），包含7列：

```
# timestamp gyro_x gyro_y gyro_z accel_x accel_y accel_z
0.0000 0.001 -0.002 0.003 0.05 0.02 9.81
0.0100 0.002 -0.001 0.004 0.06 0.03 9.80
...
```

**列说明：**
1. `timestamp`: 时间戳（秒）
2. `gyro_x/y/z`: 三轴角速度（rad/s，机体坐标系）
3. `accel_x/y/z`: 三轴比力（m/s²，机体坐标系，加速度计读数）

**注意：**
- 加速度计测量的是比力（specific force），即 f = a - g
- 静止时，加速度计读数应约为重力加速度（9.81 m/s²）向上
- 以`#`开头的行会被忽略（注释行）

## 输出结果

### 1. 控制台输出

```
[1/4] 读取IMU数据...
  数据点数: 1000
  时间范围: 0.00 - 10.00 秒

[2/4] 初始化INS解算器...
  初始位置: [0. 0. 0.]
  初始速度: [0. 0. 0.]
  初始姿态（欧拉角，度）: [0. 0. 0.]

[3/4] 执行INS积分...
  进度: 10%
  进度: 20%
  ...
  进度: 100%
  ✓ INS积分完成
  最终位置: [100.5  20.3   0.5]
  位移范围: X[0.0, 100.5] m
            Y[0.0, 20.3] m
            Z[-0.5, 0.5] m

[4/4] 生成可视化图表...
  ✓ 图表已保存: results/pure_ins/pure_ins_trajectory.png
  ✓ 结果已保存: results/pure_ins/pure_ins_trajectory.txt
```

### 2. 保存的文件

**results/pure_ins/pure_ins_trajectory.txt**
- 包含完整的轨迹数据
- 格式：timestamp pos_x pos_y pos_z vel_x vel_y vel_z roll pitch yaw

**results/pure_ins/pure_ins_trajectory.png**
- 包含6个子图的可视化结果：
  1. 2D轨迹（俯视图）
  2. 位置随时间变化
  3. 速度随时间变化
  4. 姿态角随时间变化
  5. 3D轨迹
  6. 速度大小随时间变化

### 3. Python返回值

`run_pure_ins()` 函数返回一个字典：

```python
{
    'timestamp': [t0, t1, t2, ...],              # 时间戳列表
    'position': np.array([[x0,y0,z0], ...]),     # 位置数组 (N×3)
    'velocity': np.array([[vx0,vy0,vz0], ...]),  # 速度数组 (N×3)
    'attitude_quat': np.array([[qw,qx,qy,qz], ...]),  # 四元数姿态 (N×4)
    'attitude_euler': np.array([[roll,pitch,yaw], ...])  # 欧拉角姿态 (N×3, 弧度)
}
```

## 初始状态设置

默认初始状态为：原点、静止、水平姿态

如需自定义初始状态，可在Python中调用时指定：

```python
import numpy as np

initial_state = {
    'attitude': np.array([1.0, 0.0, 0.0, 0.0]),  # 四元数[qw, qx, qy, qz]
    'velocity': np.array([10.0, 0.0, 0.0]),      # 速度[vx, vy, vz] m/s
    'position': np.array([100.0, 50.0, 0.0])     # 位置[x, y, z] m
}

trajectory = run_pure_ins(
    imu_file='data/imu.txt',
    initial_state=initial_state
)
```

## 示例：快速测试

使用测试数据快速测试：

```bash
# 生成测试数据并运行
python test/test_io_module.py

# 使用生成的测试数据运行纯INS
python runners/run_pure_ins.py --imu data/test_imu.txt --output results/test_ins
```

## 常见问题

### Q1: 位置漂移很大怎么办？
**A:** 纯INS会累积误差导致漂移，这是正常现象。解决方法：
- 使用GNSS辅助（松耦合/紧耦合ESKF，后续步骤实现）
- 检查IMU数据质量和零偏校准
- 缩短解算时间

### Q2: 加速度计数据单位如何确定？
**A:** 静止时，加速度计读数应约为9.81 m/s²（向上）。如果差异很大，可能是：
- 单位错误（如使用了g而不是m/s²）
- 坐标系方向错误
- 传感器校准问题

### Q3: 如何转换欧拉角和四元数？
**A:** 代码中已提供转换函数：
```python
from ins.mechanization import INSMechanization

# 欧拉角转四元数
quat = INSMechanization.euler_to_quaternion(roll, pitch, yaw)

# 四元数转欧拉角（通过INS对象）
ins = INSMechanization(initial_state)
euler = ins.get_euler_angles()  # 返回[roll, pitch, yaw]
```

### Q4: 坐标系约定是什么？
**A:** 
- **导航坐标系**: ENU（东-北-天）或NED（北-东-地）
- **机体坐标系**: 右手坐标系
- **重力**: ENU坐标系中为[0, 0, -9.81]，NED中为[0, 0, 9.81]
- **姿态**: Roll-Pitch-Yaw（ZYX旋转顺序）

## 技术细节

### 算法流程
1. 姿态更新：四元数微分方程积分
2. 速度更新：比力转导航系 + 重力补偿 + 积分
3. 位置更新：速度积分

### 积分方法
- 一阶欧拉法（简单但有累积误差）
- 可改进为龙格-库塔法以提高精度

### 性能
- 处理速度：约10万点/秒（取决于硬件）
- 内存占用：与数据量成正比

## 进阶使用

### 与GNSS融合（后续步骤）
纯INS结果可作为后续ESKF融合的基础：
- 第5步：松耦合ESKF（INS + GNSS导航解）
- 第6步：紧耦合ESKF（INS + GNSS原始观测）

### 批量处理
```python
import glob

# 处理多个数据文件
imu_files = glob.glob('data/imu_*.txt')
for imu_file in imu_files:
    output_dir = f'results/{Path(imu_file).stem}'
    run_pure_ins(imu_file, output_dir)
```
