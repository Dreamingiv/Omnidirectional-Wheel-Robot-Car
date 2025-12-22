# 遥控器模块

## 1. 模块简介
本模块封装了DT7类、FSI6X类和VT13类，用于接收对应遥控器的数据。


## 2. 使用前准备
### 2.1 CubeMX 配置要求
| 项目  | 配置要求 |
|---------|-----------|
| UART5 mode | Asynchronous |
| Baud Rate | 100000Bit/s |
| Word Length | 9Bits(including Parity) |
| Parity | Even |
| Stop Bits | 1 |
| UART5_RX | DMA1 STREAM 7 |
| UART5 global interrupt| Enabled |
| Preemption Priority | 5 |
| 时钟 | 120MHz |

### 2.2 依赖
- UART驱动
- C++17 编译环境


## 3. 遥控器数据格式
### 3.1 DT7
```cpp
struct RCData {
    struct {
        int16_t rocker_r_h;   // 右水平（horizontal）
        int16_t rocker_r_v;   // 右竖直（vertical）
        int16_t rocker_l_h;   // 左水平
        int16_t rocker_l_v;   // 左竖直
        int16_t dial;  // 拨轮
        uint8_t sw_left;
        uint8_t sw_right;
    } rc;

    struct {
        int16_t x;
        int16_t y;
        int16_t z;//鼠标滚轮
        uint8_t press_l[2];
        uint8_t press_r[2];
    } mouse;

    Key_t key[4];//分别为：当前按下的键，上一帧没按但这一帧按了的键，Ctrl组合键，Shift组合键
    //uint8_t key_count[3][16];//统计总按下次数的键，但是看起来没什么用
};
```
### 3.1 VT13
```cpp
struct RCData {
    float left_horizontal = 0;
    float left_vertical = 0;
    float right_horizontal = 0;
    float right_vertical = 0;
    SwitchStatus switch_A = SwitchStatus::HIGH;
    SwitchStatus switch_B = SwitchStatus::HIGH;
    SwitchStatus switch_C = SwitchStatus::HIGH;
    SwitchStatus switch_D = SwitchStatus::HIGH;
    float knob_left = 0;
    float knob_right = 0;
};
```
### 3.1 VT13
```cpp
struct RCData {
    struct {
        // 注意：VT13 的通道顺序（相对 DT7/DBUS 有差异）：
        // ch0: 右水平, ch1: 右竖直, ch2: 左竖直, ch3: 左水平
        int16_t rocker_r_h;   // ch0 - 右水平
        int16_t rocker_r_v;   // ch1 - 右竖直
        int16_t rocker_l_v;   // ch2 - 左竖直
        int16_t rocker_l_h;   // ch3 - 左水平
        int16_t dial;         // 拨轮（同样 11 位，范围同上）
        uint8_t mode_switch;  // 0/1/2 -> C/N/S
        uint8_t pause;        // 0/1
        uint8_t btn_left;     // 自定义左 0/1
        uint8_t btn_right;    // 自定义右 0/1
        uint8_t trigger;      // 扳机 0/1
    } rc;

    struct {
        int16_t x;
        int16_t y;
        int16_t z;                 // 滚轮速度（有正负）
        uint8_t press_l[2];        // [PRESS, CLICK]
        uint8_t press_r[2];
        uint8_t press_m[2];
    } mouse;

    Key_t key[4]; // [PRESS, CLICK, PRESS_WITH_CTRL, PRESS_WITH_SHIFT]
};
```


## 4. 使用方法
### 4.1 初始化
```cpp
switch (instance.remote_type_) {
    case RemoteType::VT13:
        VT13::init();
        break;
    case RemoteType::DT7:
        DT7::init();
        break;
    case RemoteType::FS_I6X:
        FSI6X::init();
        break;
    default:
        break;
}
```
### 4.2 获取DT7遥控器发送的数据
```cpp
DT7 &dt7 = DT7::getInstance();
dt7.start();
if (!dt7.isOnline()) {
	return;
}
auto &data = dt7.getData();
```

## 5. API接口
### 5.1 类静态接口
| 接口 | 功能描述 |
|----------------|------------------------------|
| `static getInstance()` | 获取实例 |
| `static init()` | 使用默认配置初始化（默认串口 5） |
| `static init(const UARTInstance::Config &config)` | 使用外部配置初始化 |
| `static init(UART_HandleTypeDef *huart)` | 使用 huart 指针初始化 |
### 5.2 实例接口
| 接口 | 功能描述 |
|-------------------------|----------------------|
| `start()` | 开始接收数据 |
| `stop（）` | 停止接收数据 |
| `getData()` | 获取完整数据 |
| `isFrameValid（）` | 判断数据是否有效 |
| `isOnline()` | 判断遥控器是否在线 |


## 6. 注意事项


## 7.更新日志
| 日期         | 更新内容        |
|------------|-------------|
| 2025-11-23 | 补全说明文档      |