# 实验运行脚本说明

本目录包含各种实验的主流程脚本。

## 脚本列表

### 1. run_pure_ins.py
纯INS惯导解算实验，仅使用IMU数据进行位置、速度、姿态积分。

**运行方式：**
```bash
python runners/run_pure_ins.py
```

### 2. run_loose_coupling.py
松耦合ESKF实验，使用GNSS导航解修正INS。

**运行方式：**
```bash
python runners/run_loose_coupling.py
```

### 3. run_tight_coupling.py
紧耦合ESKF实验，使用GNSS原始观测（伪距、伪距率）进行融合。

**运行方式：**
```bash
python runners/run_tight_coupling.py
```

**可选参数：**
- 启用/禁用约束处理

### 4. run_full_experiment.py
完整对比实验，运行所有方案并生成对比结果。

**运行方式：**
```bash
python runners/run_full_experiment.py
```

**输出内容：**
- 纯INS vs 松耦合 vs 紧耦合精度对比
- GNSS遮挡前/中/后的误差分析
- 论文用的图表和表格

## 配置文件

所有脚本都可以通过配置文件指定参数：
```bash
python runners/run_full_experiment.py --config config/custom_config.yaml
```

如果不指定配置文件，将使用默认配置 `config/default_config.yaml`。

## 输出目录

实验结果默认保存在 `results/` 目录下，包括：
- 融合轨迹数据
- 误差统计表格
- 可视化图表（PNG格式）
- 实验报告（TXT/CSV格式）
