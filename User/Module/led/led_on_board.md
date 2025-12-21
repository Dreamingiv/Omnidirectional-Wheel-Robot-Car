# LED模块

## 1. 模块简介
本模块封装了LEDOnBoard类，可控制板载LED。


## 2. 使用前准备
### 2.1 CubeMX 配置要求
| 项目  | 配置要求 |
|---------|-----------|
| SPI6 mode | Transmit Only Master |
| Hardware NSS Signal | Disable |
| Data Size | 8 Bits |
| Prescaler(for Baud Rate) | 4 |
| Clock Polarity(CPOL) | LOW |
| Clock Phase(CPHA) | 2 Edge |
| PA5/7 Maximum output speed | Very High |
| 时钟 | 24MHz |

### 2.2 依赖
- SPI驱动
- C++17 编译环境


## 3. 配置说明
### 3.1 配置结构体
使用初始化函数时，若不传入参数（见使用方法4.1），则使用默认配置
默认配置：
```cpp
SPIInstance::Config cfg = {
    .handle = &hspi6,
    .cs_port = nullptr,
    .cs_pin = 0,
    .effect_pin_state_ = GPIO_PIN_RESET,
    .type = SPIInstance::BLOCK,
    .mode = SPIInstance::TX_ONLY,
    .is_master = true,
    .tx_size = 24,
    .rx_size = 0,
    .rx_cbk = nullptr,
    .tx_cbk = nullptr
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
### 4.1 初始化（使用默认配置）
```cpp
LEDOnBoard::init();
```
### 4.2 初始化（使用自定义的SPIInstance Config）
```cpp
SPIInstance::Config cfg = {
    //...
};
LEDOnBoard::init(cfg);
```
### 4.3 设置灯光颜色（RGB格式）
```cpp
LEDOnBoard::setColorRGB(uint8_t r, uint8_t g, uint8_t b);
```
### 4.4 设置灯光颜色（HSV格式）
```cpp
LEDOnBoard::setColorHSV(const float h, const float s, const float v);
```
### 4.5 呼吸灯
```cpp
for (;;) {
    LEDOnBoard::loop();
    vTaskDelay(pdMS_TO_TICKS(10));
}
```


## 5. API接口
### 5.1 类静态接口
| 接口                | 功能描述    |
|-------------------|---------|
| `static getInstance()` | 获取LEDOnBoard类的实例 |
| `static init()` | 初始化（使用默认配置） |
| `static init(config)` | 初始化（使用自定义的SPIInstance Config） |
| `static setColorRGB(r, g, b)` | 设置灯光颜色（RGB格式） |
| `static setColorHSV(h, s, v)` | 设置灯光颜色（HSV格式） |


## 6. 注意事项


## 7.更新日志
| 日期         | 更新内容        |
|------------|-------------|
| 2025-11-20 | 补全说明文档      |
| 2025-05-28 | 初版 LED 模块实现 |
