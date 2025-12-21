# IMU（惯性测量单元）模块

## 0. 待完成功能
- 目前强制使用默认外设，之后改成可以用config提供
- 固定参数初始化滤波器 todo 考虑将其设置为带默认值的config


## 1. 模块简介
本模块封装了通用 IMU 基类，可派生出不同具体型号 (如 BMI088 / MPU6050)，用于读写加速度计和陀螺仪外设，融合或解算姿态。


## 2. 使用前准备
### 2.1 CubeMX 配置要求
| 项目  | 配置要求 |
|---------|-----------|
| SPI2 mode | Full Duplex Master |
| Hardware NSS Signal | Disable |
| Data Size | 8 Bits |
| Prescaler(for Baud Rate) | 32 |
| Clock Polarity(CPOL) | HIGH |
| Clock Phase(CPHA) | 2 Edge |
| 时钟 | 120MHz |
### 2.2 依赖
- SPI驱动
- C++17 编译环境


## 3. 配置说明
### 3.1 配置结构体
典型配置1：BMI088配置
```cpp
IMU::Config imu_config = {
    .type = IMU::Type::BMI088,
    .filter_type = IMU::FilterType::EKF,
    .sample_time = 0.001
};
```
### 3.2 配置参数说明
| 参数  | 说明 | 可选值 | 注意事项 |
|-----|----|-----|------|
| `type` | IMU 设备类型 | BMI088 / MPU6050等 | 默认为板载的BMI088 |
| `filter_type` | 姿态融合滤波器类型 | EKF / MADGWICK |  |
| `sample_time` | 采样时间 | 默认0.001 |  |

## 4. 使用方法
### 4.1 初始化
```cpp
IMU::Config imu_config{
    .type = IMU::Type::BMI088,
    .filter_type = IMU::FilterType::EKF,
    .sample_time = 0.001,
};
BMI088 bmi088(imu_config);
```
### 4.2 获取坐标并打印
```cpp
bmi088.update();
bmi088.compute();
auto result = bmi088.getQuaternion();
logger_printf("%f,%f,%f,%f\r\n", result.w, result.x, result.y, result.z);
```


## 5. API接口
### 5.1 类静态接口
| 接口             | 功能描述                     |
|----------------|------------------------------|
| `static create(config)` | 工厂函数，供上层调用 |
### 5.2 实例接口
| 接口                       | 功能描述                 |
|-------------------------|----------------------|
| `BMI088(config)` | 创建实例 |
| `init()` | 初始化传感器 |
| `calibrate()` | 零偏/校准 |
| `update()` | 更新原始数据（加速度、角速度） |
| `compute()` | 融合或解算姿态 |
| `getQuaternion()` | 获取坐标 |
| `getEulerAngles()` | 获取欧拉角 |
| `getGyro()` | 获取陀螺仪数据 |
| `getAccel()` | 获取加速度 |


## 6. 注意事项


## 7.更新日志
| 日期         | 更新内容        |
|------------|-------------|
| 2025-11-15 | 补全说明文档      |
| 2025-10-13 | 初版 IMU 模块实现 |
