# DWT 精确定时模块（driver_dwt）

## 1. 模块简介
本模块封装了 Cortex-M 内核内置的 DWT（Data Watchpoint and Trace）CYCCNT 计数器，提供高精度时间获取与延时功能。

主要设计目的为：
- 提供高精度系统运行时间轴
- 提供可靠的微秒级延时函数（不依赖中断 & 系统 Tick）
- 获取循环时间 `Δt` 用于控制算法（如 PID）
- 替代 `HAL_Delay()` 和 `osDelay()` 在实时循环中的低精度时间计算

核心功能：
- 微秒级/毫秒级/秒级时间戳
- 高精度延时（不受中断影响）
- 周期计算（常用于控制循环）


## 2. 使用前准备

### 2.1 CubeMX 配置要求

| 项目    | 配置要求                      |
|-------|---------------------------|
| CPU    | Cortex-M7，支持 DWT |
| Speculation default mode | Enabled |
| 时钟源 | 正确配置 CPU 主频（480 MHz）     |
| 中断    | 与本模块无直接依赖（不必开启）           |

### 2.2 依赖
- CMSIS Core (ARM 内核寄存器访问)
- C++17 编译环境


## 3. 配置说明

### 3.1 配置结构体

本模块无显式配置结构体，仅需在初始化时传入 CPU 主频：
```cpp
DWTInstance::init(CPU_Freq_mHz);
```

### 3.2 配置参数说明

| 参数             | 说明               |
|----------------|------------------|
| `CPU_FREQ_mHz` | CPU 主频，用于时间换算     |


## 4. 使用方法

### 4.1 初始化

```cpp
DWTInstance::init(480);   // CPU 主频 = 480 MHz
```
**注意：** 必须在主程序开始时调用一次。
调用后 CYCCNT 开始计数，时间轴正式生效。

### 4.2 获取当前系统时间（运行时间轴）

```cpp
float t_s  = DWTInstance::getTimeline_s();    // 秒
float t_ms = DWTInstance::getTimeline_ms();   // 毫秒
uint64_t t_us = DWTInstance::getTimeline_us(); // 微秒
```

### 4.3 精确定时延迟

```cpp
DWTInstance::delay(0.0005f); // 延时 0.5ms（500µs）
```
- DWT延时函数,单位为秒/s
- 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用

### 4.4 获取两次调用之间的时间间隔

```cpp
// 单位为秒/s
float dt = DWTInstance::getDeltaT();
double dt64 = DWTInstance::getDeltaT64();   // DWTGetDeltaT()的双精度浮点版本
```


## 5. API 接口

### 类静态接口

| 接口                           | 功能描述                    |
|------------------------------|-------------------------|
| `init(uint32_t CPU_Freq_mHz)` | 初始化 DWT，传入 CPU 主频（MHz）  |
| `isInitialized()` | 判断 DWT 是否已初始化           |
| `getTimeline_ms()` | 获取当前运行时间（毫秒）            |
| `delay(float Delay)` | 精确定时延时（秒）               |
| `getDeltaT()` | 获取两次调用的时间间隔（秒）          |
| `getDeltaT64()` | `DWTGetDeltaT()`的双精度浮点版本 |


## 6. 注意事项
- `init()`必须执行一次，否则全部接口无意义
- 本模块延时不受中断是否开启的影响,可以在临界区和关闭中断时使用
- 如果系统动态切换 CPU 主频，需重新调用 `init()`
- `getDeltaT()` 适用于控制循环，不要重复调用，否则 dt 变小


## 7. 更新日志

| 日期         | 更新内容        |
|------------|-------------|
| 2025-11-10 | 补全说明文档      |


