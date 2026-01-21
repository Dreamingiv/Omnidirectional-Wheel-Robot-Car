# 电机模块

## 1. 模块简介
本模块是电机封装类，有 Motor 基类和 DJIMotor DMMotor 两个派生类，内置 CAN 实例与电机进行通信。

## 2. 整体结构
采用注册表机制管理全局电机实例。
每初始化一个电机实例，会在对应类别电机的实例注册表 `std::array<> motor_` 中注册。

工作流程上，
主要分为在 `motor_task` 里运行的 `controlAll()` 函数和在 CAN 中断回调中运行的 `parseData()` 函数。

`controlAll()` 函数目前是 `sendCommandAll()` 的封装，实质上是遍历注册表中的所有元素并发送 CAN 命令。

`parseData()` 函数在 CAN 中断回调中执行，是将接收到的数据帧解码并存到 `measure_` 变量中。

*若用户需要加入自己编写的其他电机类，请最好也按照这个结构来。*

## 3. 使用方法

### 3.1 初始化
使用工厂函数 `create()` 和 Config 结构体进行构造：
```c++
// DJI
using ega::Motor;
Motor::Config motor_config;
motor_config.type = Motor::Type::GM6020;
motor_config.can_config = {
    .handle = &hfdcan1
};
motor_config.dji_motor_config = {
    .motor_id = 1,
    .reduction_radio = 1.0f
};
motor_config.direction = Motor::Direction::NORMAL;

//std::unique_ptr<Motor> motor =  Motor::create(motor_config);
 auto motor = Motor::create(motor_config);
```
```c++
// DM
using ega::Motor;
Motor::Config motor_config;
motor_config.type = Motor::Type::DM_MOTOR;
motor_config.can_config = {
    .handle = &hfdcan2,
    .tx_id = 0x01,
    .rx_id = 0x11,
};
motor_config.dm_motor_config = {
    .p_max_abs = 12.5f,
    .v_max_abs = 45.0f,
    .t_max_abs = 18.0f,
    .reduction_radio = 1.0f,
    .use_mit = false,
};

//std::unique_ptr<Motor> motor =  Motor::create(motor_config);
 auto motor = Motor::create(motor_config);
```

### 3.2 设定输出
使用 `setEffort()` 函数，用该函数输入的值为电流或者力矩，直观来说就是设置电机的“力”的大小。

该函数仅会更改输出缓存变量 `effort_` 的值，实际发送输出命令要等到 `motor_task` 中的 `sendCommand()` 指令。

```c++
motor->setEffort(3000);
```

若使用达妙电机的 MIT 模式，即在 `config` 中设置 `use_mit = True`，则可使用 `setMIT()` 函数。

```c++
// void setMIT(float position, float velocity, float torque,float kp,float kd)
dm_motor->setMIT(30.0f, 0, 0, 10.0f, 5.0f);
```

> 这里的类内变量 effort_ 具体来说指的是“控制电机输出力度”的指令值，在不同类别的电机中所代指的物理量有所不同，
> 
> 在 DJI 电机中，输入的值为电流（3508、2006、流控6020）/电压（压控6020）对应的 bit 值，其值域为 ±25000（压控） 或 ±16384（流控）；
> 
> 在不用 MIT 的 DM 电机中，输入的值为扭矩值，其值域为设定的 ±t_max_abs_。

### 3.3 获取测量值
大部分情况下调取电机通用的 measure 即可。推荐用 `getMeasure()` 方法并存至局部变量中使用。

```c++
auto measure = motor->getMeasure();
measure.total_angle;    // 【输出轴】上电后经过的完整角度
measure.speed;          // 【输出轴】速度 度每秒
measure.torque;         // 等效力矩
```

若要调取 DJI/DM 电机特有的测量值，使用 `getDMMeasure()` 和 `getDJIMeasure()` 方法。

```c++
auto dji_measure = dji_motor->getDJIMeasure();
auto dm_measure = dm_motor->getDMMeasure();
```

### 3.4 其余方法
通用方法：

|        方法名称         |       描述        |
|:-------------------:|:---------------:|
|      `create`       |     创建电机实例      |
|     `setEffort`     |  设置电机输出“力”的大小   |
|    `unsetEffort`    |  将力设为0，停止电机的输出  |
| `setMotorDirection` |    调整电机旋转方向     |
|      `enable`       |      启用电机       |
|      `disable`      |      禁用电机       |
|     `isEnabled`     |    查询电机启用状态     |
|     `isOnline`      |    查询电机在线状态     |
|    `isEffortSet`    | 查询电机是否设置effort  |    
|    `getMeasure`     |    获取电机通用测量值    |
|     `getEffort`     |   获取当前effort值   |
|   `getEffortMax`    | 获取该电机effort的最大值 |
|       `debug`       |   打印电机内部变量的接口   |

DJI 电机专有：

|          方法名称          |       描述        |
|:----------------------:|:---------------:|
|    `getDJIMeasure`     | 获取 DJI 电机专有的测量值 |
| `getDJIReductionRadio` | 获取当前 DJI 电机的减速比 |

DM 电机专有：

|       方法名称        |          描述           |
|:-----------------:|:---------------------:|
|     `setMIT`      | 在 MIT 模式下设置 MIT 目标和参数 |
| `setZeroPosition` |     将当前位置设置为0点位置      |
|  `getDMMeasure`   |    获取 DM 电机专有的测量值     |

以上为所有用户可以调用的方法，其余未提到的均 **不建议** 用户自己调用。

## 4. 与 Controller 联合使用

通常情况下电机需要与控制器同时使用。这里建议用数组存储电机实例的控制器，并通过串级输入的形式进行计算，例程如下：

```c++
class Foo
{
    std::unique_ptr<Motor> motor_[4] {};
    std::unique_ptr<Controller> motor_pidc_[4][2] {};

    Foo()
    {
        Motor::Config motor_config;
        motor_config.type = Motor::Type::M3508;
        motor_config.can_config = {
            .handle = &hfdcan1
        };
        for (uint8_t i = 0; i < 4; i++)
        {
            motor_config.dji_motor_config = {
                .motor_id = static_cast<uint8_t>(1 + i),
                .reduction_radio = 1.0f
            };
            motor_config.direction = Motor::Direction::NORMAL;
            motor_[i] = Motor::create(motor_config);
        }
        Controller::Config controller_config;
        controller_config.type = Controller::Type::PID;
        controller_config.pid_config = {
            .kp = 1.0f, ...
        };
        motor_pidc_[0][0] = Controller::create(controller_config);
        ...
    }

    void positionControl()
    {
        auto measure = motor_[0]->getMeasure();
        float target = 90; // deg
        float ref = target;
        ref = motor_pidc_[0][0]->calculate(ref, measure.total_angle, 0);
        ref = motor_pidc_[0][1]->calculate(ref, measure.speed, 0);
        float output = ref;
        motor_[0]->setEffort(output);
        ...
    }
};
```

> 这里的示例仅说明基本用法，具体结构需要结合实际自己探索。


## 5. TODO
- 6020流控版尚未完成。
- DM 电机未实现实时计算电流。
