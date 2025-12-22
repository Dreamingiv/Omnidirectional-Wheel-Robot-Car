# 控制器模块

## 1. 模块简介
本模块封装了控制器的相关功能，提供pid控制和滑模控制两种算法，用于计算电机的输出值。


## 2. 使用前准备
### 2.1 依赖
- C++17 编译环境


## 3. 配置说明
### 3.1 配置结构体
典型配置1：pid控制
```cpp
Controller::Config pid_config = {
	.kp = 0,
	.ki = 0,
	.kd = 0,
	.limit_error = 0.0f,
	.limit_integral = 0.0f,
	.limit_diff = 0.0f,
	.limit_output = 1e6f,
	.dead_band = 0.0f,
	.use_ramp = false,
	.ramp_step = 0.0f,
	.dt = 2  //ms
};
```
典型配置2：滑模控制
```cpp
Controller::Config smc_config = {
	.lambda = 1.0f,
	.k = 2.0f,
	.phi = 0.02f,
	.k_soft = 0.5f,
	.phi_soft = 0.1f,
	.err_switch = 0.1f,
	.limit_output = 1e6f,
	.dead_band = 0.0f,
	.use_ramp = false,
	.ramp_step = 0.0f,
	.dt = 2.0f
};
```
### 3.2 配置参数说明
| 参数  | 说明   | 可选值  | 注意事项 |
|-----|------|------|------|
| `kp` | 比例增益 | float  |      |
| `ki` | 积分增益 | float  |      |
| `kd` | 微分增益 | float  |      |
| `limit_error` | 误差限幅 | float |      |
| `limit_integral` | 积分限幅 | float |      |
| `limit_diff` | 微分限幅 | float |      |
| `limit_output` | 输出限幅 | float |      |
| `dead_band` | 死区范围 | float |      |
| `use_ramp` | 是否启用斜坡函数 | bool |      |
| `ramp_step` | 斜坡速率 | float |      |
| `dt` | 时间间隔 | float |      |
| `lambda` | 滑模面系数 | float |      |
| `k` | 大误差区开关增益 | float |      |
| `phi` | 大误差区开关增益 | float |      |
| `k_soft` | 小误差时较弱的开关增益 | float |      |
| `phi_soft` | 小误差时更大的边界层厚度 | float |      |
| `err_switch` | 误差切换阈值 | float |      |


## 4. 使用方法
### 4.1 创建控制器
```cpp
controller = Controller::create(Config config);
```
### 4.2 计算输出值
```cpp
output = controller->calculate(float target, float measure, float forward);
```
### 4.3 清空输出值和积分项
```cpp
controller->clear();
```

## 5. API接口
### 5.1 类静态接口

| 接口                | 功能描述    |
|-------------------|---------|
| `static create(config)` | 创建控制器 |

### 5.2 实例接口

| 接口                | 功能描述    |
|-------------------|---------|
| `calculate(target, measure, forward)` | 计算电机的输出值 |
| `clear()` | 清空输出值和积分项 |


## 6. 注意事项



## 7.更新日志

| 日期         | 更新内容        |
|------------|-------------|
| 2025-11-10 | 补全说明文档      |
| 2025-10-18 | 初版控制器模块实现 |
