# 测试模块

本目录包含所有模块的测试脚本。

## 测试文件列表

### test_config.py
配置管理器测试

**测试内容：**
- 加载默认配置
- 验证配置完整性
- 获取各类参数（数据路径、IMU参数、GNSS参数、ESKF参数）
- 嵌套键值获取
- 配置保存和加载

**运行方式：**
```bash
python test/test_config.py
```

### test_io_module.py
数据IO模块测试（完整版）

**测试内容：**
- IMU数据读取器（IMUReader）
- GNSS数据读取器（GNSSReader）
- 时间对齐器（TimeAligner）
- 自动生成测试数据
- 数据对齐和遮挡标记
- 自动清理测试数据

**运行方式：**
```bash
python test/test_io_module.py
```

**测试数据：**
- 自动生成10秒测试数据
- IMU: 1000个数据点@100Hz
- GNSS导航解: 11个历元@1Hz
- GNSS原始观测: 88个观测（8颗卫星×11历元）
- 模拟遮挡: 3-5秒和7-8秒

### test_io.py
数据IO模块测试（旧版）

**说明：** 与test_io_module.py功能重复，建议使用test_io_module.py

## 运行所有测试

```bash
# 配置测试
python test/test_config.py

# IO模块测试
python test/test_io_module.py

# 后续会添加更多测试
# python test/test_ins.py
# python test/test_eskf.py
```

## 测试数据

测试脚本会自动在 `data/` 目录生成测试数据，测试完成后自动清理。

## 注意事项

1. **导入问题**：由于Python内置的`io`模块命名冲突，测试脚本使用`importlib`动态加载模块
2. **路径问题**：所有测试脚本从项目根目录运行
3. **数据清理**：测试数据会自动清理，不会留下临时文件

## 添加新测试

创建新测试文件时，参考现有测试的结构：

```python
"""
模块测试脚本
"""

from pathlib import Path
import sys

# 设置项目根目录
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# 导入待测试模块
# ...

def test_function_name():
    """测试某个功能"""
    print("测试 XXX 功能...")
    # 测试代码
    pass

def main():
    """主测试函数"""
    print("=" * 60)
    print("模块名称测试")
    print("=" * 60)
    
    test_function_name()
    
    print("\n所有测试完成！")

if __name__ == "__main__":
    main()
```
