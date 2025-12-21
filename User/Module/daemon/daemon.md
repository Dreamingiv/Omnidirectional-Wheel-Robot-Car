# 守护进程模块

## 1. 模块简介
本模块封装了守护进程（Daemon）类，用于监控模块是否卡死或离线。每个模块可注册一个实例，在RTOS任务中周期性调用Tick()。


## 2. 使用前准备
### 2.1 依赖
- 第三方库FreeRTOS
- C++17 编译环境


## 3. 配置说明
### 3.1 配置结构体
典型配置1：使用情况1
```cpp
DaemonInstance::Config config = {
    .reload_count = 100,
    .init_count = 100,
    .callback = nullptr
};
```

### 3.2 配置参数说明
| 参数  | 说明 | 可选值 | 注意事项 |
|-----|----|-----|------|
| `reload_count` | 超时时间计数 | uint16_t | 默认100 |
| `init_count` | 初始计数 | uint16_t | 可用于上电延迟 |
| `callback` | 离线时的回调函数 | 函数指针 / nullptr |  |


## 4. 使用方法
### 4.1 定义掉线回调函数
```cpp
void offlineCallback() {
	//...
}
```
### 4.2 实例化守护进程
```cpp
std::optional<DaemonInstance> daemon_;
daemon_.emplace(DaemonInstance::Config{
    .reload_count = 10,
    .init_count   = 10,
    .callback     = [this]() { this->offlineCallback(); }
});
```
### 4.3 计数
```cpp
DaemonInstance::tick();//所有已注册的守护进程tick一次
vTaskDelay(pdMS_TO_TICKS(10)); // 例如在RTOS中每10ms调用一次
```


## 5. API接口
### 5.1 类静态接口
| 接口             | 功能描述                          |
|----------------|------------------------------------|
| `static tick()` | 周期任务，用于所有实例计数递减与离线检测 |
### 5.2 实例接口
| 接口                       | 功能描述                 |
|-------------------------|----------------------|
| `DaemonInstance(config)` | 创建实例，注册到静态实例表 |
| `~DaemonInstance()` | 注销自身，保持数组连续 |
| `reload()` | “喂狗”——重载计数 |
| `isOnline()` | 是否在线 |

## 6. 注意事项
- 最大监控对象数量由`MX_DAEMON_NUM`决定，为32，使用时请注意不要超出。


## 7.更新日志
| 日期         | 更新内容        |
|------------|-------------|
| 2025-11-15 | 补全说明文档      |
| 2025-10-3 | 初版 daemon 模块实现 |

