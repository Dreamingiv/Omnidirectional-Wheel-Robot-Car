# Omnidirectional Wheel Robot Car

基于 RoboMaster C 型开发板的四全向轮 45 度布置底盘工程。当前代码用于一台 PWM + DIR 控制的全向轮小车，支持 DT7 遥控器控制、CAN2 上位机速度控制、yaw hold、速度斜坡和底盘调试输出。

本仓库基于 `RM2026_EGAdapter_Cboard_base` 二次开发，保留了原工程的 driver/module/application/task 分层。

## 当前功能

- 四全向轮底盘混控
- XYT-300 控制器 PWM + DIR 开环控制
- DT7/DR16 遥控器输入
- CAN2 上位机速度指令输入
- 遥控器/上位机/保护模式仲裁
- `wz = 0` 时基于 IMU yaw 的航向保持
- 普通速度斜坡，减少速度突变
- `Chassis::debug_printf()` 底盘调试输出

## 硬件连接

### 电机 PWM / DIR

| 轮子 | PWM | DIR |
| --- | --- | --- |
| LF 左前 | TIM1_CH1 / PE9 | PC6 |
| RF 右前 | TIM1_CH2 / PE11 | PI6 |
| LB 左后 | TIM1_CH3 / PE13 | PI7 |
| RB 右后 | TIM1_CH4 / PE14 | PB12 |

### CAN2 上位机接口

| 信号 | 引脚 |
| --- | --- |
| CAN2_RX | PB5 |
| CAN2_TX | PB6 |

### DT7 / DR16

| 信号 | 引脚 |
| --- | --- |
| USART3_RX | PC11 |
| USART3_TX | PC10 |

## 控制模式

### DT7 拨杆逻辑

| 右拨杆 | 左拨杆 | 模式 |
| --- | --- | --- |
| DOWN | 任意 | Protect |
| MIDDLE | MIDDLE | 遥控器控制 |
| MIDDLE | UP | 上位机控制 |
| MIDDLE | DOWN | Protect |
| UP | 任意 | 预留，目前 Protect |

如果 DT7 离线，但 CAN2 上位机数据有效且 `enable = 1`，底盘会使用上位机控制。否则进入 Protect，PWM 输出为 0。

### 坐标定义

- `vx > 0`：车体向前
- `vy > 0`：车体向左
- `wz > 0`：旋转正方向以当前实车调试结果为准

## CAN2 上位机协议

该协议是当前临时版本，后续可能根据上位机需求调整。

- CAN 外设：CAN2
- 帧类型：标准帧
- StdId：`0x301`
- DLC：8
- 超时：200 ms 未收到有效帧则认为上位机离线

### 数据帧格式

| Byte | 名称 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | magic | `uint8_t` | 固定 `0xA5` |
| 1 | version | `uint8_t` | 固定 `0x01` |
| 2 | flags | `uint8_t` | bit0 = enable，bit1 = reset_yaw_target |
| 3 | vx | `int8_t` | `vx = raw / 100.0f` |
| 4 | vy | `int8_t` | `vy = raw / 100.0f` |
| 5 | wz | `int8_t` | `wz = raw / 100.0f` |
| 6 | seq | `uint8_t` | 帧序号 |
| 7 | checksum | `uint8_t` | Byte0 到 Byte6 按位 XOR |

### 示例帧

目标速度：

```text
enable = 1
reset_yaw_target = 0
vx = 0.50
vy = 0.00
wz = 0.00
seq = 0x10
```

完整数据：

```text
A5 01 01 32 00 00 10 87
```

校验：

```text
0xA5 ^ 0x01 ^ 0x01 ^ 0x32 ^ 0x00 ^ 0x00 ^ 0x10 = 0x87
```

## 调试输出

`Chassis::debug_printf()` 会限频输出底盘状态，目前约 200 ms 打印一次：

```text
CHASSIS mode=... cmd=(vx,vy,wz) wheel=(LF,RF,LB,RB) effort=(LF,RF,LB,RB) max=...
```

如果要修改打印内容或频率，改 `User/Application/chassis/chassis.cpp` 中的 `Chassis::debugPrintfImpl()`。

如果要关闭底盘调试输出，注释 `User/Task/robot_task/robot_task.cpp` 中的：

```cpp
Chassis::debug_printf();
```

## 测试模式

`User/Task/robot_task/robot_task.cpp` 中定义了：

```cpp
#define CHASSIS_TEST_MODE
```

打开时会进入测试模式，按前、停、后、停、左、停、右、停循环运行。正式使用遥控器或上位机控制时，需要注释掉这一行。

## 构建

```bash
cmake --build cmake-build-debug
```

## 注意事项

- 当前底盘是 PWM 开环控制，没有轮速闭环。
- IMU 横向积分补偿当前不启用，只保留 yaw hold。
- XYT-300 控制器黄色 CAP 速度反馈线如果是 5V 脉冲，不能直接接 STM32 IO，需要先做 3.3V 电平转换。
- 使用 CAN2 上位机控制时，需要确保 CAN 总线接线、终端电阻和共地正确。

