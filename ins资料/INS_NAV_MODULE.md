# 嵌入式惯导模块使用说明

## 1. 模块位置

当前嵌入式惯导模块位于：

- `Core/Inc/ins_nav.h`
- `Core/Src/ins_nav.c`

这是一个适合 STM32 这类平台部署的纯 C 惯导机械化模块，不依赖：

- MATLAB
- 文件读写
- Eigen
- 动态内存

它的职责只有一件事：

- 接收传感器每个采样周期输出的数据
- 在内部完成惯导机械化更新
- 输出当前位置、速度、姿态和四元数


## 2. 模块适配的传感器输出格式

你当前的传感器直接输出：

- 三轴加速度
- 三轴角速度
- 三轴磁场
- 欧拉角
- 四元数

因此模块对外推荐使用的输入接口是 `INS_SensorFrame`，定义在 [ins_nav.h](Core/Inc/ins_nav.h)。

```c
typedef struct
{
    ins_real_t timestamp;      /* s */
    ins_real_t dt;             /* s */
    INS_Vec3 accel_mps2;       /* 三轴加速度 / 比力 */
    INS_Vec3 gyro_rad_s;       /* 三轴角速度 */
    INS_Vec3 mag;              /* 三轴磁场 */
    INS_Attitude angle;        /* 传感器直接输出的角度 */
    INS_Quaternion quaternion; /* 传感器直接输出的四元数 */
} INS_SensorFrame;
```

注意：

- 这 6 组数据都会作为接口输入保留下来
- 当前纯惯导机械化核心只真正使用：
  - `accel_mps2`
  - `gyro_rad_s`
- `mag / angle / quaternion` 当前不参与机械化解算
- 它们保留下来是为了接口和你传感器输出一致，方便后续接磁力计约束、姿态观测或 EKF


## 3. 每个输入字段是什么意思

### `timestamp`

- 当前采样时刻
- 单位：秒 `s`
- 必须严格递增

### `dt`

- 当前采样周期
- 单位：秒 `s`
- 必须大于 0

### `accel_mps2`

- 三轴加速度计输出
- 单位：`m/s²`
- 在惯导里，它应当理解为加速度计测得的比力

注意这里最容易搞错。  
惯导里用的不是“已经被你外部处理过的线加速度”，而是 IMU 加速度计原始意义上的测量量。

### `gyro_rad_s`

- 三轴角速度
- 单位：`rad/s`

### `mag`

- 三轴磁场
- 单位由你的传感器决定
- 当前纯惯导模块不使用

### `angle`

- 传感器直接给出的欧拉角
- 单位建议统一成 `rad`
- 当前纯惯导模块不使用

### `quaternion`

- 传感器直接给出的姿态四元数
- 当前纯惯导模块不使用


## 4. 模块内部真正需要的量是什么

虽然你的传感器输出的是角速度和加速度，但惯导机械化真正使用的不是速率，而是“每个采样周期的增量”：

- 角增量
\[
\Delta \theta = \omega \cdot dt
\]

- 速度增量
\[
\Delta v = f \cdot dt
\]

也就是说，模块内部会自动做这件事：

- `gyro_delta = gyro_rad_s * dt`
- `acc_delta = accel_mps2 * dt`

然后再把这两个增量送入惯导机械化内核。

这就是为什么你现在不需要手工再去构造 `INS_ImuSample` 了。


## 5. 输出是什么

模块输出结构体是 `INS_State`：

```c
typedef struct
{
    ins_real_t timestamp;
    INS_Blh blh;
    INS_Velocity velocity;
    INS_Attitude attitude;
    INS_Quaternion quaternion;
} INS_State;
```

输出字段含义如下：

- `blh.latitude`：纬度，单位 `rad`
- `blh.longitude`：经度，单位 `rad`
- `blh.height`：高度，单位 `m`
- `velocity.vn`：北向速度，单位 `m/s`
- `velocity.ve`：东向速度，单位 `m/s`
- `velocity.vd`：地向速度，单位 `m/s`
- `attitude.roll / pitch / yaw`：姿态角，单位 `rad`
- `quaternion`：当前机械化解算得到的姿态四元数

这里输出的是内部导航状态，不是“显示格式”。  
如果你要串口输出角度制，需要你自己再做 `rad -> deg` 转换。


## 6. 坐标和符号约定

模块内部导航坐标系使用 `NED`：

- `N`：North，北
- `E`：East，东
- `D`：Down，地

因此速度定义是：

- `vn`：北向速度
- `ve`：东向速度
- `vd`：向下速度

也就是说：

- 向下运动时 `vd > 0`
- 向上运动时 `vd < 0`

这件事必须提前想清楚。  
如果你的机体坐标或上层算法习惯用 `ENU`，那你在喂数据前必须统一成模块当前约定。


## 7. 初始化怎么做

### 第一步：准备配置

```c
INS_Config cfg;
INS_DefaultConfig(&cfg);
```

### 第二步：设置初始状态

至少需要设置：

- 初始位置
- 初始速度
- 初始姿态

示例：

```c
INS_Config cfg;
INS_DefaultConfig(&cfg);

cfg.timestamp = 0.0f;

cfg.initial_blh.latitude  = lat0_rad;
cfg.initial_blh.longitude = lon0_rad;
cfg.initial_blh.height    = h0_m;

cfg.initial_velocity.vn = 0.0f;
cfg.initial_velocity.ve = 0.0f;
cfg.initial_velocity.vd = 0.0f;

cfg.initial_attitude.roll  = roll0_rad;
cfg.initial_attitude.pitch = pitch0_rad;
cfg.initial_attitude.yaw   = yaw0_rad;
```

### 第三步：初始化上下文

```c
INS_Context ins;
INS_Init(&ins, &cfg);
```


## 8. 正常调用接口

你现在推荐使用的是：

```c
INS_Status INS_UpdateSensorFrame(INS_Context* ctx,
                                 const INS_SensorFrame* frame,
                                 uint8_t zero_velocity,
                                 INS_State* out_state);
```

这是当前最贴合你传感器输出的高层接口。


## 9. 最小调用示例

```c
#include "ins_nav.h"

static INS_Context g_ins;
static INS_State g_nav;

void App_INS_Init(void)
{
    INS_Config cfg;

    INS_DefaultConfig(&cfg);

    cfg.initial_blh.latitude  = lat0_rad;
    cfg.initial_blh.longitude = lon0_rad;
    cfg.initial_blh.height    = h0_m;

    cfg.initial_velocity.vn = 0.0f;
    cfg.initial_velocity.ve = 0.0f;
    cfg.initial_velocity.vd = 0.0f;

    cfg.initial_attitude.roll  = roll0_rad;
    cfg.initial_attitude.pitch = pitch0_rad;
    cfg.initial_attitude.yaw   = yaw0_rad;

    INS_Init(&g_ins, &cfg);
}

void App_INS_Step(float timestamp,
                  float dt,
                  float ax, float ay, float az,
                  float gx, float gy, float gz,
                  float mx, float my, float mz,
                  float roll, float pitch, float yaw,
                  float q0, float q1, float q2, float q3)
{
    INS_SensorFrame frame;

    frame.timestamp = timestamp;
    frame.dt = dt;

    frame.accel_mps2.x = ax;
    frame.accel_mps2.y = ay;
    frame.accel_mps2.z = az;

    frame.gyro_rad_s.x = gx;
    frame.gyro_rad_s.y = gy;
    frame.gyro_rad_s.z = gz;

    frame.mag.x = mx;
    frame.mag.y = my;
    frame.mag.z = mz;

    frame.angle.roll = roll;
    frame.angle.pitch = pitch;
    frame.angle.yaw = yaw;

    frame.quaternion.q0 = q0;
    frame.quaternion.q1 = q1;
    frame.quaternion.q2 = q2;
    frame.quaternion.q3 = q3;

    if (INS_UpdateSensorFrame(&g_ins, &frame, 0U, &g_nav) == INS_STATUS_OK)
    {
        /* 此时 g_nav 就是当前惯导解 */
    }
}
```


## 10. 为什么前几帧不马上出结果

模块内部不是只用当前一帧数据，它还要保存前两帧增量样本。

因此运行过程是：

1. 第 1 个样本：缓存
2. 第 2 个样本：缓存
3. 第 3 个样本开始：才能稳定输出当前导航解

所以前两次调用可能返回：

- `INS_STATUS_NEED_MORE_DATA`

这是正常行为。


## 11. 零速约束怎么用

接口中的 `zero_velocity` 参数含义如下：

- `0`
  - 正常机械化更新

- 非 `0`
  - 当前时刻强制把速度置零
  - 再继续做位置和姿态更新

适用场景：

- 你上层已经做了零速检测
- 当前可以确定设备静止

注意：

- 模块本身不包含自动零速检测
- 它只提供“外部告诉它现在静止”的入口


## 12. 偏置补偿怎么用

模块支持简单的固定偏置补偿：

```c
typedef struct
{
    uint8_t enable_acc_bias_correction;
    uint8_t enable_gyro_bias_correction;
    INS_Vec3 acc_bias_delta;
    INS_Vec3 gyro_bias_delta;
} INS_BiasConfig;
```

注意这里的单位不是速率单位，而是“每采样周期的增量偏置”：

- `acc_bias_delta` 单位要和内部 `acc_delta` 一致，即 `m/s`
- `gyro_bias_delta` 单位要和内部 `gyro_delta` 一致，即 `rad`

也就是说，如果你手上拿到的是：

- 加速度 bias：`m/s²`
- 角速度 bias：`rad/s`

那么必须先乘当前采样周期 `dt`，再填进 bias 配置。


## 13. 模块内部数据是怎么一步步变化的

每调用一次 `INS_UpdateSensorFrame()`，内部数据变化如下。

### 第一步：读取传感器帧

输入帧中有：

- 三轴加速度
- 三轴角速度
- 三轴磁场
- 欧拉角
- 四元数

但当前纯惯导核心只取：

- `accel_mps2`
- `gyro_rad_s`

### 第二步：速率变增量

内部自动计算：

- `gyro_delta = gyro_rad_s * dt`
- `acc_delta = accel_mps2 * dt`

这一步之后，数据从“速率/加速度形式”变成了“惯导机械化需要的增量形式”。

### 第三步：速度更新

模块使用：

- 当前和前两帧 IMU 增量
- 上一时刻和上上时刻导航状态

内部做：

- 划桨修正
- 地球自转补偿
- 运输角速度补偿
- 重力项补偿
- 哥氏项补偿

输出新的：

- `vn`
- `ve`
- `vd`

### 第四步：可选零速修正

如果 `zero_velocity != 0`：

- 当前速度直接强制为 0

### 第五步：位置更新

使用新的 `vn ve vd` 更新：

- 纬度
- 经度
- 高度

### 第六步：姿态更新

使用陀螺增量更新：

- 四元数
- 欧拉角

内部包含：

- 圆锥修正
- 导航系旋转补偿

### 第七步：输出导航状态

最终输出：

- `blh`
- `velocity`
- `attitude`
- `quaternion`


## 14. 使用时最容易出错的地方

### 14.1 角速度单位错

模块要求输入 `gyro_rad_s`，单位必须是：

- `rad/s`

如果你传的是：

- `deg/s`

就必须先转成 `rad/s`。

### 14.2 角度单位错

如果你把传感器输出的 `angle` 填进去了，建议统一成：

- `rad`

虽然当前模块不使用它，但后面如果加观测融合，单位混乱会直接出问题。

### 14.3 把线加速度当比力

当前模块假设 `accel_mps2` 是加速度计原始测量意义上的量。  
如果你在外部已经做了错误的重力补偿，再喂给模块，速度会明显漂。

### 14.4 机体系轴定义没对齐

你的传感器输出轴可能不等于算法机体系。

如果不先做轴交换和符号翻转：

- 姿态会错
- 速度会错
- 位置会错

### 14.5 时间戳不递增

如果 `timestamp` 不递增，模块会返回：

- `INS_STATUS_INVALID_TIMESTAMP`

### 14.6 `dt` 填错

如果 `dt` 比实际值大或小：

- 角增量会错
- 速度增量会错
- 整个导航都会错

所以 `dt` 不是一个普通字段，而是这套算法能不能工作的关键量。


## 15. 当前模块哪些字段没真正参与解算

当前版本中：

- `mag`
- `angle`
- `quaternion`

都没有进入机械化主链。

这不是 bug，而是当前模块的目标决定的：

- 现在它是“纯惯导机械化核心”
- 不是“多传感器融合姿态系统”

这些字段保留下来的意义是：

- 让接口和你的传感器输出一致
- 为后续扩展 EKF、航向约束、姿态观测做准备


## 16. 这套算法目前还缺什么

作为纯惯导机械化，它目前还缺这些工程能力：

- 自动零速检测
- EKF 误差状态滤波
- 磁场观测融合
- 传感器欧拉角/四元数观测融合
- GNSS 融合
- 在线零偏估计
- 初始自动对准

所以它现在适合做的是：

- 纯惯导预测内核
- 短时间姿态、速度、位置递推
- 作为后续 EKF 的预测部分

它现在不适合单独承担：

- 长时间高精度导航
- 长时间独立抗漂移运行


## 17. 推荐的上层系统结构

建议你的工程结构按下面分层：

1. 传感器驱动层
   - 读取加速度、角速度、磁场、角度、四元数
2. 预处理层
   - 时间同步
   - 单位转换
   - 坐标轴转换
3. 惯导机械化层
   - `INS_UpdateSensorFrame()`
4. 辅助判定层
   - 零速检测
5. 融合层
   - EKF / ZUPT / 磁场约束 / GNSS
6. 应用层
   - 控制
   - 显示
   - 串口输出


## 18. 结论

现在这套模块已经改成了适配你当前传感器输出的接口。

你可以直接把一帧传感器数据打包成 `INS_SensorFrame`，然后调用：

- `INS_UpdateSensorFrame()`

它内部会自动把：

- 三轴角速度
- 三轴加速度

转换成惯导机械化需要的增量，再完成位置、速度、姿态更新。

如果你后面继续做工程化，最值得加的两项是：

1. 自动零速检测
2. 误差状态 EKF

