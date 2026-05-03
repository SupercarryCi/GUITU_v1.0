# GUITU RTOS Framework

## 启动链路

1. `main.c` 完成 CubeMX 生成的 GPIO/DMA/UART/ADC/I2C/SPI 基础初始化。
2. `MX_FREERTOS_Init()` 调用 `App_RtosCreateObjects()`。
3. `App_RtosCreateObjects()` 创建事件组、队列、互斥锁、ADC 信号量、全局状态互斥锁，只启动 `InitTask`。
4. `InitTask` 调用各模块 `Task_*InitHardware()`，全部成功后创建真实业务任务。
5. `InitTask` 设置 `SYS_EVT_INIT_DONE`，业务任务解除阻塞并开始运行。

## 任务分工

- `GyroTask`: USART1 ReceiveToIdle DMA 接收陀螺仪数据，队列入口为 `g_gyroRxQueue`，解析 hook 为 `App_GyroDecodeFrame()`，解析成功后调用 `INS_UpdateSensorFrame()` 更新导航状态。
- `UiTask`: 周期刷新显示并轮询触摸，显示 hook 为 `App_UiRender()`，触摸 hook 为 `App_UiPollTouch()`，用户操作输出到 `g_uiCmdQueue`。
- `ControlTask`: 消费 `g_uiCmdQueue`，把返航类用户指令转发到 `g_returnCmdQueue`。
- `ReturnTask`: 默认阻塞，收到返航开始指令后进入运行态，周期调用 `App_ReturnStep()`。
- `AdcTask`: 周期启动 ADC1 DMA 采两路数据，DMA 完成回调释放 `g_adcReadySem`，结果写入全局状态。
- `Spo2Task`: 周期通过 I2C 读取血氧，读数 hook 为 `App_Spo2ReadSample()`。
- `LoraTask`: 默认预留不启用，队列已建好；启用 `APP_LORA_ENABLE_DEFAULT` 后最高 5Hz 轮询/发送。
- `DebugTask`: 消费 `g_debugLogQueue`，通过 USART2 输出 `App_DebugLog()`。

## 同步对象

- 事件组: `g_sysEventFlags`
- 队列: `g_gyroRxQueue`, `g_uiCmdQueue`, `g_loraTxQueue`, `g_loraRxQueue`, `g_returnCmdQueue`, `g_debugLogQueue`
- 互斥锁: `g_spiDisplayMutex`, `g_spiTouchMutex`, `g_spiLoraMutex`, `g_i2cBusMutex`, `g_debugUartMutex`
- 信号量: `g_adcReadySem`

## 全局状态

所有对外共享数据都集中在 `app_state.c`，通过 `App_StateSet*()` / `App_StateGet*()` 访问：

- `GyroState_t`: 最新陀螺仪原始帧、解析计数、INS 状态码
- `NavState_t`: 惯导输出位置、速度、姿态、四元数
- `AdcState_t`: 两路 ADC 原始值和毫伏值
- `Spo2State_t`: 血氧、心率、灌注指数
- `UiState_t`: 最新用户指令和 UI 计数
- `LoraState_t`: LoRa 收发统计和最近接收包
- `ReturnState_t`: 返航状态机
- `AppSnapshot_t`: UI/返航读取整机快照的结构

## 需要填业务代码的 hook

- `App_GyroDecodeFrame()` in `task_gyro.c`
- `App_UiHardwareInit()`, `App_UiRender()`, `App_UiPollTouch()` in `task_ui.c`
- `App_Spo2HardwareInit()`, `App_Spo2ReadSample()` in `task_spo2.c`
- `App_LoraHardwareInit()`, `App_LoraTransmit()`, `App_LoraPollRx()` in `task_lora.c`
- `App_ReturnOnStart()`, `App_ReturnStep()`, `App_ReturnOnStop()` in `task_return.c`

建议后续把这些 hook 的真实实现放到新的驱动/业务文件中，不要直接堆回任务文件。
