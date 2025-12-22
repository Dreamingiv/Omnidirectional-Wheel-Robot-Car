# 电机模块

## 1. 模块简介
本模块封装了Motor父类和DJIMotor、DMMotor子类，用于控制大疆电机（GM6020、M3508、M2006）或达妙电机。


## 2. 使用前准备
### 2.1 依赖
- C++17 编译环境


## 3. 配置说明
### 3.1 配置结构体
典型配置1：GM6020电机
```cpp
Motor::Config config = {
    .motor_type = Motor::MotorType::GM6020,
    .motor_setting= {
        .work_mode= Motor::WorkMode::POSITION,
        .loop_type=Motor::LoopType::POSITION_AND_VELOCITY,
    },
    .position_controller_config{
        .type=Controller::Type::PID,
        .pid_config{
            .kp=5,
            .ki=0,
            .kd=0.5
            },
    },
    .velocity_controller_config{
        .type=Controller::Type::PID,
        .pid_config{
            .kp=5,
            .ki=0.5,
            .kd=0,
            .limit_output=10000,
        }
    },
    .can_config{
        .can_handle=&hfdcan1
    },
    .dji_motor_config{
        .motor_id=1,
        .reduction_radio = 0.0f
    }
};
```
典型配置2：达妙电机
```cpp
Motor::Config config = {
    .motor_type=Motor::MotorType::DM_MOTOR,
    .motor_setting{
        .work_mode=Motor::WorkMode::POSITION,
    },
    .can_config={
        .can_handle=&hfdcan2,
        .tx_id=0x01,
        .rx_id=0x00
    },
    .dm_motor_config{
        .use_mit = true,
        .kp=5,
        .kd=0.5,
        .p_max_abs=12.5,
        .v_max_abs=30,
        .t_max_abs=10
    }
};
```
### 3.2 配置参数说明
| 参数  | 说明 | 可选值 | 注意事项 |
|-----|----|-----|------|
| `motor_type` | 电机种类 | OTHER/GM6020/M3508/M2006/DM_MOTOR |  |
| `work_mode` | 工作模式 | NONE/OPEN/POSITION/VELOCITY/TORQUE | 默认为位置控制 |
| `loop_type` | 闭环类型 | OPEN/POSITION/VELOCITY/CURRENT/POSITION_AND_VELOCITY/ALL_THREE |  |
| `position/velocity/current_controller_config` | 三环控制器设置 |  |  |
| `can_config` | CAN通信接口设置 |  |  |
|-----|----|-----|------|
| `dji_motor_config` | 大疆电机专有的初始化配置 |  |  |
| `motor_id` | 大疆电机编号 | 从1到8 | 必须提供 |
| `reduction_radio` | 减速比 | float | 无减速箱的写1：1 |
|-----|----|-----|------|
| `dm_motor_config` | 达妙电机专有的初始化配置 |  |  |
| `use_mit` | 是否使用mit模式控制 | true / false | 默认为true，否则使用三环控制 |


## 4. 使用方法
### 4.1 初始化实例
```cpp
DJIMotor dji_motor({
    .motor_type = Motor::MotorType::GM6020,
    .motor_setting= {
        .work_mode= Motor::WorkMode::POSITION,
        .loop_type=Motor::LoopType::POSITION_AND_VELOCITY,
    },
    .position_controller_config{
        .type=Controller::Type::PID,
        .pid_config{
            .kp=5,
            .ki=0,
            .kd=0.5
            },
    },
    .velocity_controller_config{
        .type=Controller::Type::PID,
        .pid_config{
            .kp=5,
            .ki=0.5,
            .kd=0,
            .limit_output=10000,
        }
    },
    .can_config{
        can_handle=&hfdcan1
    },
    .dji_motor_config{
        .motor_id=1,
    }
});
dji_motor.setRef(0);
```
### 4.2 电机控制循环
```cpp
Motor::enableAll();
for(;;){
    Motor::controlLoop();
    vTaskDelay(pdMS_TO_TICKS(2));//电机控制周期500hz
}
```


## 5. API接口
### 5.1 类静态接口
| 接口 | 功能描述 |
|----------------|------------------------------|
| `static create(config)` | 工厂函数，方便上层创建电机 |
| `static controlLoop()` | 所有电机的总控制循环 |
| `static enableAll()` | 使能所有电机 |
| `static disableAll()` | 禁用所有电机 |
### 5.2 实例接口
| 接口 | 功能描述 |
|-------------------------|----------------------|
| `decodeData(*data, len)` | 数据解析 |
| `setRef(ref)` | 目标值更新，Application层主要调用setRef |
| `updateControl()` | 计算电机的输出值 |
| `sendCommand()` | 发送控制命令 |


## 6. 注意事项


## 7.更新日志
| 日期         | 更新内容        |
|------------|-------------|
| 2025-11-23 | 补全说明文档      |
