# INS/GNSS 融合系统

车载 MEMS IMU + 双天线 GNSS 姿态测量离线处理系统

## 项目结构

```
GNSS_IMU_fusion_algorithm/
├── config/          # 配置文件管理
├── io/              # 数据读取与时间对齐
├── ins/             # 纯惯导解算
├── models/          # 状态定义、误差方程、IMU/GNSS模型
├── fusion/          # ESKF融合（松耦合&紧耦合）、约束
├── eval/            # 精度评估与绘图
├── runners/         # 实验主流程脚本
└── data/            # 数据存放目录
```

## 功能模块

- **纯INS惯导解算**：仅依靠IMU进行位置、速度、姿态积分
- **松耦合ESKF**：使用GNSS导航解修正INS
- **紧耦合ESKF**：使用GNSS原始观测（伪距、伪距率）进行融合
- **约束处理**：基线长度约束、零速约束等
- **遮挡实验**：模拟GNSS遮挡场景，评估系统鲁棒性
- **精度评估**：全面的误差分析和可视化

## 安装依赖

```bash
pip install -r requirements.txt
```

## 使用方法

详见各模块说明和 `runners/` 目录下的实验脚本
