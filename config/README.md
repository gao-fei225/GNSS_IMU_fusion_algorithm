# 配置管理模块

本模块负责管理INS/GNSS融合系统的所有配置参数。

## 文件说明

- **config_manager.py**: 配置管理器类，负责读取、验证和提供配置参数
- **default_config.yaml**: 默认配置文件，包含基本参数设置
- **example_config.yaml**: 示例配置文件，包含更详细的配置选项和注释
- **test_config.py**: 配置管理器测试脚本

## 使用方法

### 1. 使用默认配置

```python
from config import ConfigManager

# 使用默认配置
config = ConfigManager()

# 获取各类参数
imu_params = config.get_imu_params()
gnss_params = config.get_gnss_params()
eskf_params = config.get_eskf_params()
```

### 2. 加载自定义配置

```python
from config import ConfigManager

# 加载自定义配置文件
config = ConfigManager('path/to/custom_config.yaml')

# 获取数据路径
data_paths = config.get_data_paths()
print(f"IMU数据: {data_paths['imu_data']}")
```

### 3. 获取特定配置值

```python
# 使用嵌套键获取值
sample_rate = config.get_value('imu_params.sample_rate')
attitude_cov = config.get_value('eskf_params.initial_cov.attitude')

# 提供默认值
custom_param = config.get_value('custom.param', default=0.5)
```

### 4. 验证配置

```python
# 验证配置完整性
is_valid, errors = config.validate_config()
if not is_valid:
    for error in errors:
        print(f"配置错误: {error}")
```

### 5. 打印配置信息

```python
# 格式化打印所有配置
config.print_config()
```

### 6. 保存配置

```python
# 保存当前配置到文件
config.save_config('output/my_config.yaml')
```

## 配置结构

配置文件采用YAML格式，包含以下主要部分：

### data_paths（数据路径）
- `imu_data`: IMU原始数据文件路径
- `gnss_nav`: GNSS导航解文件路径（松耦合）
- `gnss_raw`: GNSS原始观测文件路径（紧耦合）
- `reference`: 参考轨迹文件路径（评估用）
- `output_dir`: 结果输出目录

### imu_params（IMU参数）
- `sample_rate`: 采样频率（Hz）
- `gyro_noise`: 陀螺噪声（rad/s/√Hz）
- `accel_noise`: 加速度噪声（m/s²/√Hz）
- `gyro_bias_walk`: 陀螺零偏随机游走（rad/s²/√Hz）
- `accel_bias_walk`: 加计零偏随机游走（m/s³/√Hz）

### gnss_params（GNSS参数）
- `output_rate`: 输出频率（Hz）
- `position_noise`: 位置噪声（m）
- `velocity_noise`: 速度噪声（m/s）
- `pseudorange_noise`: 伪距噪声（m）
- `doppler_noise`: 多普勒噪声（m/s）

### eskf_params（ESKF参数）
- `state_dim`: 误差状态维数（通常为15）
- `initial_cov`: 初始协方差设置
  - `attitude`: 姿态协方差（rad²）
  - `velocity`: 速度协方差（m²/s²）
  - `position`: 位置协方差（m²）
  - `gyro_bias`: 陀螺零偏协方差（rad²/s²）
  - `accel_bias`: 加计零偏协方差（m²/s⁴）

### experiment（实验设置）
- `enable_loose_coupling`: 是否启用松耦合
- `enable_tight_coupling`: 是否启用紧耦合
- `enable_constraints`: 是否启用约束
- `gnss_outages`: GNSS遮挡时间段列表
- `constraints`: 约束配置
  - `baseline_length`: 基线长度（m）
  - `enable_zupt`: 是否启用零速更新
  - `zupt_threshold`: 零速检测阈值（m/s）

## 配置示例

### 最小配置示例

```yaml
data_paths:
  imu_data: "data/imu.txt"
  output_dir: "results/"

imu_params:
  sample_rate: 100.0
  gyro_noise: 0.0001
  accel_noise: 0.001

gnss_params:
  output_rate: 1.0

eskf_params:
  state_dim: 15
  initial_cov:
    attitude: 0.01
    velocity: 0.1
    position: 1.0

experiment:
  enable_loose_coupling: true
  enable_tight_coupling: true
```

### 完整配置示例

参见 `example_config.yaml` 文件。

## 参数调节建议

### IMU噪声参数
根据传感器数据手册设置：
- **低成本MEMS**：gyro_noise ≈ 0.001, accel_noise ≈ 0.01
- **中等MEMS**：gyro_noise ≈ 0.0001, accel_noise ≈ 0.001
- **高精度IMU**：gyro_noise ≈ 0.00001, accel_noise ≈ 0.0001

### GNSS精度参数
根据接收机精度设置：
- **单点定位**：position_noise ≈ 1-5 m
- **差分定位**：position_noise ≈ 0.1-1 m
- **RTK定位**：position_noise ≈ 0.01-0.1 m

### ESKF初始协方差
- **姿态不确定**：attitude ≈ 0.1 (约18°)
- **姿态较确定**：attitude ≈ 0.01 (约5.7°)
- **姿态精确已知**：attitude ≈ 0.001 (约1.8°)

### 遮挡时间段
根据实际场景设置，例如：
- **城市峡谷**：多段短时遮挡（10-30秒）
- **隧道**：长时间遮挡（30-120秒）
- **立交桥下**：极短遮挡（5-10秒）

## 测试

运行测试脚本验证配置管理器：

```bash
python config/test_config.py
```

测试内容包括：
- 配置文件加载
- 参数获取
- 配置验证
- 配置保存
- 格式化输出

## 注意事项

1. **路径格式**：使用相对路径或绝对路径，Windows下注意路径分隔符
2. **参数单位**：严格按照注释中的单位设置
3. **配置验证**：修改配置后建议运行 `validate_config()` 验证
4. **备份配置**：重要配置建议备份，避免误修改
5. **版本管理**：不同实验使用不同配置文件，便于结果复现
