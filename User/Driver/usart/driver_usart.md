# UART 通信模块（driver_usart）

## 1. 模块简介
本模块封装了 STM32 HAL UART 外设的驱动，提供面向对象的串口通信接口。

主要设计目的为：
- 为 DMA / 中断 / 阻塞 三种模式提供统一的 UART 抽象层
- 支持发送队列（FIFO）与内部数据复制池
- 支持 IDLE 中断接收（DMA/IT）
- 多实例管理（最多 6 个 UART）
- 自动管理 HAL 回调，无需用户自行在回调里写分发逻辑

核心功能：
- 阻塞 / DMA / 中断三种发送模式
- 可选发送队列（FIFO）
- 支持 `sendCopy()` 自动管理数据生命周期
- 空闲中断（IDLE）自动回调
- 多实例自动注册与回调分发

## 2. 使用前准备

### 2.1 CubeMX 配置要求

| 项目                      | 配置要求                 |
|-------------------------|----------------------|
| UART Mode               | Asynchronous         |
| Baud Rate               | 100000               |
| Word Length             | 9 Bits               |
| Parity                  | Even                 |
| UART5_RX                | 启用DMA1 Stream7       |
| UART5 global interrrupt | Enabled              |
| 时钟                      | 120MHz               |


### 2.2 依赖
- HAL UART 驱动
- C++17 编译环境


## 3. 配置说明

### 3.1 配置结构体

典型配置：绑定一个 TX ID 和一个 RX ID，并绑定一个函数名作为接收回调
```cpp
UARTInstance::Config config = {
    .handle     = &huart5,
    .tx_type    = UARTInstance::DMA,
    .rx_type    = UARTInstance::IT_IDLE,
    .rx_cbk     = nullptr,
    .rx_size    = 0,
    .use_fifo   = true,
    .queue_mx_size = 4,
};
```

### 3.2 配置参数说明

| 参数        | 说明       | 可选值                                                        | 注意事项                     |
|-----------|----------|------------------------------------------------------------|--------------------------|
| `handle`  | HAL 串口句柄 | `&huart5/7`                                                | 必填                       |
| `tx_type` | 发送模式     | `DMA / IT / BLOCK`                                         | BLOCK 不允许使用 FIFO         |
| `rx_type` | 接收模式     | `BLOCK / DMA / IT + NUM / IDLE` | 若是非阻塞模式，会自动启动接收          |
| `tx_cbk`  | 发送回调函数   | 函数指针 / `nullptr`                                           | 无参数                      |
| `rx_cbk`  | 接收回调函数   | 函数指针 / `nullptr`                                           | 参数`(uint8_t*, uint16_t)` |
| `rx_size` | 每包接收长度   | uint16_t                                                   | NUM 模式下必须设置              |
|   `use_fifo`        |    是否开启发送队列      | `true / false`                                             | DMA/IT 推荐开启              |
|     `queue_mx_size`      |    FIFO 长度      | uint8_t                                                    | 默认 4                     |


## 4. 使用方法

### 4.1 定义回调

```cpp
void onUartRx(uint8_t* data, uint16_t len) {
    // UART 接收到数据自动回调
}
```

### 4.2 初始化实例

```cpp
UARTInstance uart1({
    .handle     = &huart5,
    .tx_type    = UARTInstance::DMA,
    .rx_type    = UARTInstance::IT_IDLE,
    .rx_cbk     = onUartRx,
    .rx_size    = 64,
    .use_fifo   = true,
    .queue_mx_size = 4,
});
```

### 4.3 发送数据

#### 指针发送
```cpp
uint8_t buf[] = "Hello UART";
uart1.send(buf, sizeof(buf));
```

#### 拷贝发送
避免用户缓冲区生命周期问题：
```cpp
const uint8_t msg[] = {1,2,3,4};
uart1.sendCopy(msg, sizeof(msg));
```

### 4.4 阻塞接收

```cpp
uint8_t rxbuf[10];
uint16_t real = uart1.recv(rxbuf, 10, 100);
```


## 5. API 接口

### 5.1 类静态接口

| 接口                           | 功能描述                               |
|------------------------------|------------------------------------|
| `HAL_UARTEx_RxEventCallback()` | IDLE 中断接收分发 |
| `HAL_UART_TxCpltCallback()` | 发送完成回调分发、FIFO 出队 |
| `HAL_UART_RxCpltCallback()` | 普通接收完成分发 |
| `HAL_UART_ErrorCallback()` | 错误恢复接收 |

### 5.2 实例接口

| 接口                                            | 功能描述               |
|-----------------------------------------------|--------------------|
| `UARTInstance(config)`                        | 创建实例，自动注册并启动接收     |
| `send(*data, size, timeout)`                  | 指针发送（DMA/IT/BLOCK） |
| `sendCopy(*data, size, timeout)`              | 内部缓冲池发送，安全无生命周期问题  |
| `setSendType(type, use_fifo, queue_mx_size))` | 修改发送模式，清空队列        |
| `setRecvType(type, size)`                     | 切换接收模式             |
| `recv(*data, size, timeout)`                  | 阻塞接收模式             |
| `startRecv()`                                 | 重启（DMA/IT）非阻塞接收    |
| `stopRecv()`                                  | 停止接收并清空缓冲区         |


## 6. 注意事项
- BLOCK 模式不能启用 FIFO
- IDLE + DMA/IT 是推荐的高性能接收模式
- `send()` 使用指针发送，必须保证数据在发送完成前不被释放
- `sendCopy()` 自动托管数据，可连续 4 次（内部 4 块 buffer）
- 所有实例会被静态数组管理，最多 6 个实例（`UART_MX_INS_NUM`）
- 若更改发送 / 接收模式，内部会清空队列或重新启动接收


## 7. 更新日志

| 日期         | 更新内容        |
|------------|-------------|
| 2025-11-20 | 补全说明文档      |


