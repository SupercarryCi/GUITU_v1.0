# 纯惯导动态导航定位实验

## 项目简介

本项目是一个基于惯性导航原理的纯惯导动态导航定位实验系统。通过处理IMU（惯性测量单元）原始数据，实现速度、位置和姿态的实时解算，并与参考真值进行对比分析。

## 项目结构

```
.
├── 代码/                    # C++源代码目录
│   ├── main.cpp            # 主程序入口
│   ├── IMU_Structs.h       # 数据结构定义
│   ├── ReadFile.cpp        # 文件读取模块
│   ├── NavigationUpdate.cpp # 导航更新算法（速度、位置、姿态）
│   ├── CoordinateChange.cpp # 坐标转换模块
│   ├── Calibration.cpp     # IMU校准模块
│   ├── Calculations.cpp    # 计算辅助函数
│   ├── MatrixPrint.cpp     # 矩阵打印工具
│   └── SaveFile.cpp        # 结果保存模块
├── 代码/Result/            # MATLAB结果分析目录
│   ├── main.m              # MATLAB主程序
│   ├── ReadFile.m          # MATLAB文件读取
│   ├── DataFig.m           # 数据可视化
│   └── Figures/            # 生成的图表
└── 实验报告/               # 实验报告文档

```

## 主要功能

1. **数据读取**：支持二进制格式的示例数据和文本格式的自定义IMU数据
2. **导航解算**：
   - 速度更新（Velocity Update）
   - 位置更新（Position Update）
   - 姿态更新（Posture Update）
3. **数据校准**：支持加速度计和陀螺仪的零偏校准
4. **误差分析**：计算解算结果与参考真值的误差
5. **轨迹分析**：计算ENU坐标系下的位置偏差

## 使用方法

### 编译

使用支持C++11标准的编译器编译项目：

```bash
g++ -std=c++11 *.cpp -o navigation -I/path/to/eigen
```

注意：项目依赖Eigen库进行矩阵运算。

### 运行

运行编译后的可执行文件：

```bash
./navigation
```

程序会提示选择模式：
- `0`：示例数据模式（ExampleData）
- `1`：自定义数据模式（SelfData）

### 数据文件要求

**示例数据模式**：
- `Data/Example_Data/IMU.bin`：IMU原始数据（二进制格式）
- `Data/Example_Data/PureIns.bin`：参考真值数据（二进制格式）

**自定义数据模式**：
- `Data/GroupOne.ASC`：自定义IMU数据（文本格式）
- `Data/TruthOne.nav`：参考真值数据（文本格式）

### 结果输出

程序会在 `Result/` 目录下生成以下文件：
- `INS_Result.txt`：解算结果（位置、速度、姿态）
- `INS_True.txt`：参考真值
- `INS_Diff_Result.txt`：误差分析结果
- `denu_Result.txt`：ENU坐标系下的位置偏差

### MATLAB可视化

运行 `Result/main.m` 进行结果可视化：

```matlab
cd Result
main
```

## 技术特点

- 使用四元数进行姿态解算
- 采用双速度算法进行速度更新
- 支持WGS84坐标系下的位置解算
- 实现IMU零偏校准功能
- 支持零速修正（Zero Velocity Update）

## 参数配置

主要参数在 `IMU_Structs.h` 中定义：
- WGS84椭球参数
- 初始位置、速度、姿态
- IMU设备参数（采样率、标度因数等）

## 注意事项

1. 确保数据文件路径正确
2. 自定义数据模式下需要正确设置初始条件
3. 零速修正功能需要在代码中手动配置时间区间

## 作者

GYH

## 实验课程

惯性导航原理 - 实验三

