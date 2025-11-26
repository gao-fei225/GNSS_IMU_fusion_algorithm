# 数据目录

本目录用于存放IMU、GNSS和参考数据。

## 数据文件格式

### 1. IMU数据（imu_data.txt）

**格式：** CSV或空格分隔的文本文件

**列：**
```
timestamp, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z
```

- `timestamp`: 时间戳（秒）
- `gyro_x/y/z`: 三轴角速度（rad/s）
- `accel_x/y/z`: 三轴加速度（m/s²）

**示例：**
```
0.000, 0.001, -0.002, 0.003, 0.05, 0.02, 9.81
0.010, 0.002, -0.001, 0.004, 0.06, 0.03, 9.80
...
```

### 2. GNSS导航解（gnss_nav.txt）

**格式：** CSV或空格分隔的文本文件

**列：**
```
timestamp, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z
```

- `timestamp`: 时间戳（秒）
- `pos_x/y/z`: 位置（m，导航坐标系）
- `vel_x/y/z`: 速度（m/s，导航坐标系）

**示例：**
```
0.000, 100.0, 200.0, 50.0, 10.0, 0.5, 0.0
1.000, 110.0, 200.5, 50.0, 10.0, 0.5, 0.0
...
```

### 3. GNSS原始观测（gnss_raw.txt）

**格式：** CSV或空格分隔的文本文件

**列：**
```
timestamp, prn, pseudorange, pseudorange_rate, sat_x, sat_y, sat_z, sat_vx, sat_vy, sat_vz
```

- `timestamp`: 时间戳（秒）
- `prn`: 卫星号
- `pseudorange`: 伪距（m）
- `pseudorange_rate`: 伪距率/多普勒（m/s）
- `sat_x/y/z`: 卫星位置（m，ECEF坐标系）
- `sat_vx/vy/vz`: 卫星速度（m/s，ECEF坐标系）

**示例：**
```
0.000, 1, 20000000.0, 100.0, 15000000.0, 10000000.0, 20000000.0, 1000.0, 500.0, 200.0
0.000, 5, 22000000.0, -50.0, 18000000.0, 12000000.0, 18000000.0, -800.0, 600.0, 100.0
...
```

### 4. 参考真值（reference.txt）

**格式：** CSV或空格分隔的文本文件

**列：**
```
timestamp, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, qw, qx, qy, qz
```

- `timestamp`: 时间戳（秒）
- `pos_x/y/z`: 参考位置（m）
- `vel_x/y/z`: 参考速度（m/s）
- `qw/qx/qy/qz`: 参考姿态（四元数）

**示例：**
```
0.000, 100.0, 200.0, 50.0, 10.0, 0.5, 0.0, 1.0, 0.0, 0.0, 0.0
0.010, 100.1, 200.0, 50.0, 10.0, 0.5, 0.0, 0.999, 0.001, 0.002, 0.001
...
```

## 坐标系说明

### 机体坐标系（Body Frame）
- X轴：指向机体前方
- Y轴：指向机体右侧
- Z轴：指向机体下方（右手系）

### 导航坐标系（Navigation Frame）
推荐使用ENU（东-北-天）或NED（北-东-地）坐标系。

**ENU坐标系：**
- X轴：指向东（East）
- Y轴：指向北（North）
- Z轴：指向天（Up）

**NED坐标系：**
- X轴：指向北（North）
- Y轴：指向东（East）
- Z轴：指向地（Down）

### ECEF坐标系（Earth-Centered Earth-Fixed）
- 原点：地球质心
- X轴：指向本初子午线与赤道的交点
- Z轴：指向北极
- Y轴：与X、Z轴构成右手系

## 数据要求

1. **IMU采样率**：建议≥100Hz
2. **GNSS采样率**：通常1-10Hz
3. **时间对齐**：所有数据使用统一的时间基准（如GPS时间）
4. **单位统一**：严格按照上述单位要求
5. **数据完整性**：避免数据缺失或跳变

## 测试数据

如果没有实际数据，可以使用仿真数据或公开数据集：
- [KITTI数据集](http://www.cvlibs.net/datasets/kitti/)
- [EuRoC MAV数据集](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- 自行生成仿真数据（见`tools/data_generator.py`）
