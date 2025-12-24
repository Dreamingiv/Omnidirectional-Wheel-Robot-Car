# 控制器模块

## 1. 模块简介
本模块是控制器算法封装类，有 Controller 基类和 PIDController SMCController 两个派生类，用于集成计算控制算法。

## 2. 整体结构
控制器结构较简单，实例间独立性较强。

初始化时由 Config 结构体中的 `type` 段裁决为何种控制器，然后将对应的 xxx_config 传入子类构造。

工作流上，`calculate()` 函数内进行完整的一次计算，`clear()` 函数进行对控制器的重置清零。

*若用户需要加入自己编写的其他电机类，请最好也按照这个结构来。*

## 3. 使用方法
### 3.1 初始化
使用工厂函数 `create()` 函数和 Config 结构体进行构造。

Config 结构体中包含了控制器所有需要的静态参数，调参时主要就是调这些 `config` 变量。

```c++
// PID
using ega::Controller;
Controller::Config controller_config;
controller_config.type = Controller::Type::PID;
controller_config.pid_config = {
    .kp = 1, .ki = 0, .kd = 0,
    .limit_output = 1e6f,
    .limit_integral = 1e6f,
    .limit_error = 1e6f,
    .limit_diff = 1e6f,
    .dead_band = 0,
    .use_ramp = false,
    .ramp_step = 0,
    .dt = 2,    // ms
};

//std::unique_ptr<Controller> controller = Controller::create(controller_config);
auto controller = Controller::create(controller_config);
```
```c++
// SMC
using ega::Controller;
Controller::Config controller_config;
controller_config.type = Controller::Type::SMC;
controller_config.smc_config = {
    .J = 1.0f,
    .dt = 0.005f, // s
    .k = 2.0f,
    .lambda = 1.0f,
    .dead_band = 0.05f,
    .alpha = 1.0f,
    .epsilon = 1.0f,
    .phi = 0.05f,
    .beta = 2.0f,
    .use_ramp = false,
    .ramp_step = 0.0f,
    .q = 21.0f,
    .p = 27.0f,
};


//std::unique_ptr<Controller> controller = Controller::create(controller_config);
auto controller = Controller::create(controller_config);
```

### 3.2 计算
使用 `calculate()` 方法，输入 目标值 `target`，测量值 `measure`，前馈值 `forward`。
计算后输出 结果值 `output`。

```c++
float output = controller->calculate(target, measure, forward);
```