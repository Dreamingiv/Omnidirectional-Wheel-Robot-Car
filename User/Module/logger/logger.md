# 日志模块

## 1. 模块简介
本模块封装了logger类，该类提供线程安全的日志输出功能，支持格式化字符串和DMA/中断方式的串口传输，用于输出调试信息。


## 2. 使用前准备
### 2.1 CubeMX 配置要求
| 项目  | 配置要求 |
|---------|-----------|
| USART1 mode | Asynchronous |
| USART1_RX | DMA1 STREAM 0 |
| USART1_TX | DMA1 STREAM 1 |
| USART1 global interrupt| Enabled |
| Preemption Priority | 5 |
| 时钟 | 120MHz |


### 2.2 依赖
- UART驱动
- C++17 编译环境


## 3. 配置说明
### 3.1 配置结构体
典型配置1：默认配置
```cpp
UARTInstance::Config config = {
    .handle = &huart1,
    .tx_type = UARTInstance::DMA,// 使用 DMA 发送（可按需改为 IT）
    .rx_type = UARTInstance::IT_IDLE,// 中断接收
    .rx_size = 64,// Logger 默认只发不收，如有必要后期可以拓展
    .use_fifo = true,// 使用 FIFO 缓冲
    .queue_mx_size = 4,// 队列容量，最多可连续调用4次
};
```
### 3.2 配置参数说明
| 参数  | 说明 | 可选值 | 注意事项 |
|-----|----|-----|------|
| `handle` | HAL 串口句柄 | `&huart1` | 默认使用串口1 |
| `tx_type` | 发送模式 | `DMA / IT / BLOCK` | BLOCK 不允许使用 FIFO |
| `rx_type` | 接收模式 | `BLOCK_NUM / DMA_NUM / IT_NUM / BLOCK_IDLE / DMA_IDLE / IT_IDLE` | 若是非阻塞模式，会自动启动接收 |
| `rx_size` | 每包接收长度 | uint16_t | NUM 模式下必须设置 |
| `use_fifo` | 是否开启发送队列 | `true / false` | DMA/IT模式下推荐开启 |
| `queue_mx_size` | FIFO 长度 | uint8_t | 默认 4 |

## 4. 使用方法
### 4.1 使用默认配置初始化 Logger
```cpp
Logger::init(&huart1);
```
### 4.2 使用自定义配置初始化 Logger
```cpp
UARTInstance::Config config = {
    //...
};
Logger::init(config);
```
### 4.3 输出调试信息
```cpp
logger_printf(const char *fmt, ...)
```


## 5. API接口
### 5.1 类静态接口
| 接口                | 功能描述    |
|-------------------|---------|
| `static getInstance()` | 获取Logger类的实例 |
| `static init(UART_HandleTypeDef *handle)` | 使用默认配置初始化 Logger |
| `static init(UARTInstance::Config &cfg)` | 使用自定义配置初始化 Logger |
| `static printf(const char *fmt, ...)` | 格式化输出调试信息 |
| `static vPrintf(const char *fmt, va_list args)` | va_list 版本的格式化输出，避免二次 vsnprintf |
### 5.2 实例接口
| 接口                | 功能描述    |
|-------------------|---------|
| `inline logger_printf(const char *fmt, ...)` | 全局内联函数，便于快速调用调试打印 |

## 6. 注意事项


## 7.更新日志
| 日期         | 更新内容        |
|------------|-------------|
| 2025-11-22 | 补全说明文档      |
