# SPI 通信模块（driver_spi）

## 1. 模块简介
本模块封装了 STM32 HAL SPI 外设，提供可配置的片选控制 + 多传输模式（BLOCK / DMA / IT）的面向对象 SPI 通信接口。

主要设计目的为：
- 简化 SPI 收发流程，避免直接调用 HAL 层带来的冗余与复杂性
- 支持同一个 SPI 外设对应多个实例（通过片选引脚区分不同从机）
- 驱动 IMU、Flash、Sensor 等 SPI 设备

核心功能：
- 支持 BLOCK / DMA / IT 三种传输方式
- 支持 全双工 / 半双工 / 仅发 / 仅收 四种通信模式
- 可选发送/接收完成中断回调
- 自动执行回调函数，无需手动在 HAL 回调中处理


## 2. 使用前准备

### 2.1 CubeMX 配置要求

| 项目                       | 配置要求                               |
|--------------------------|------------------------------------|
| SPI 模式                   | Full Duplex Master (常用)            |
| Data Size                | 8 bits                             |
| Prescaler(for Baud Rate) | 32                                 |
| Clock Polarity(CPOL)     | HIGH                               |
| Clock Phase(CPHA)        | 2 Edge                             |
| NSS / 片选                 | 软件管理（NSS = Software），使用 GPIO 手动控制  |
| DMA（可选）                  | 若使用 DMA 模式，则开启 SPI_TX 和 SPI_RX DMA |
| 中断（可选）                   | 若使用 IT 模式，则启用 SPI 全局中断             |
| 时钟 | SPI1,2,3(120MHz)，SPI6(24MHz) |

### 2.2 依赖
- HAL SPI 驱动
- C++17 编译环境


## 3. 配置说明

### 3.1 配置结构体

SPI 初始化配置
```cpp
SPIInstance::Config config = {
    .handle = &hspi2,
    .cs_port = ACC_CS_GPIO_Port,
    .cs_pin = ACC_CS_Pin,
    .effect_pin_state_ = GPIO_PIN_RESET,
    .type = SPIInstance::BLOCK,
    .mode = SPIInstance::FULL_DUPLEX,
    .is_master = true,
    .tx_size = 8,
    .rx_size = 8,
    .tx_cbk = nullptr,
    .rx_cbk = nullptr,
};
```

### 3.2 配置参数说明

| 参数                  | 说明         | 可选值            | 注意事项                 |
|---------------------|------------|----------------|----------------------|
| `handle`            | HAL SPI 句柄 | `&hspi1 / &hspi2 / ...` | 必填                   |
| `cs_port`, `cs_pin` | 片选引脚       | GPIOx + Pin | 必填，需用户事先初始化          |
| `effect_pin_state`  | 片选有效电平     | `GPIO_PIN_SET/RESET` | 与从设备要求一致             |
| `type`              | 传输方式       | `BLOCK / DMA / IT`  | BLOCK 最简单，DMA 最省 CPU |
| `mode`              | 工作模式       | `FULL_DUPLEX / HALF_DUPLEX / TX_ONLY / RX_ONLY`  | 需与外设一致               |
| `is_master`         | 主从设置       | `True / False` | 默认为主机                |
| `tx_size`           | 发送数据长度     | int | 可动态变更                |
| `rx_size`           | 接收数据长度     | int | 可动态变更                |
| `tx_cbk`            | 传输完成回调     | 回调函数指针 / nullptr | 用于 IT / DMA 模式       |
| `rx_cbk`            | 接收完成回调     | 回调函数指针 / nullptr | 用于 RX_ONLY 或全双工      |


## 4. 使用方法

### 4.1 定义回调

```cpp
void onSpiRx(uint8_t* data, uint16_t len) {
    // 处理接收到的数据
}
```

### 4.2 初始化实例

```cpp
SPIInstance spi1({
    .handle = &hspi2,
    .cs_port = GPIOA,
    .cs_pin = GPIO_PIN_4,
    .effect_pin_state_ = GPIO_PIN_RESET,
    .type = SPIInstance::BLOCK,
    .mode = SPIInstance::FULL_DUPLEX,
    .rx_cbk = onSpiRx,
});
```

### 4.3 收发数据

**全双工收发**
```cpp
uint8_t tx_data[3] = {0xAA, 0xBB, 0xCC};
spi1.transRecv(tx_data, 3);   // BLOCK 模式下会等待传输完成
```

**仅发送**
```cpp
uint8_t cmd = 0x01;
spi1.trans(&cmd, 1);
```

### 4.4 寄存器访问

```cpp
spi1.writeReg(0x0F, 0x55);
uint8_t value = spi1.readReg(0x0F);
```


## 5. API 接口

### 实例接口

| 接口                             | 功能描述                 |
|--------------------------------|----------------------|
| `transRecv(tx, size, timeout)` | 全双工收发 |
| `transRecv(tx, rx, size)`      | 全双工收发，接收写入用户提供缓冲区 |
| `trans(tx, size, timeout)`     | 仅发送             |
| `recv(size, timeout)`          | 仅接收      |
| `writeReg(reg, byte)`          | 向地址写一字节数据   |
| `readReg(reg)`                 | 从地址读一字节数据 |
| `byte(byte)`                   | 写一字节，读一字节 |
| `setTransRecvType(type, size)` | 切换 BLOCK / DMA / IT 模式 |
| `stop()`                       | 停止当前传输（适用于所有模式） |
| `cs_enable() / cs_disable()`   | 手动片选控制 |


## 6. 注意事项
- 同一个 SPI 外设可创建多个实例，但每个实例必须绑定独立 CS 引脚
- DMA/IT 模式下，tx_data 和 rx_data 必须在传输完成前保持有效，不可释放或离开作用域
- `SPI_MX_INS_NUM` 限制了同一外设可创建的实例数量（默认 2）
- 默认缓冲大小为 64 字节，若需要大数据传输，请修改 `SPI_MX_BUFFER_SIZE`


## 7. 更新日志

| 日期         | 更新内容        |
|------------|-------------|
| 2025-11-10 | 补全说明文档      |

