//
// Author: An
// Date: 25-10-03
//

#ifndef MODULE_DAEMON_H_
#define MODULE_DAEMON_H_

#include <cstdint>
#include <array>
#include <functional>
#include <algorithm>

namespace ega
{
    /**
     * @brief 守护进程（Daemon）实例，用于监控模块是否卡死或离线。
     *        每个模块可注册一个实例，在RTOS任务中周期性调用Tick()。
     */
    class Daemon
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        static constexpr size_t MX_INS_NUM = 32; // 最大监控对象数量
        using OfflineCallback = std::function<void()>; // 去除void*参数

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        struct Config
        {
            uint16_t reload_count = 100; // 超时时间计数（默认100）
            uint16_t init_count = 100; // 初始计数（可用于上电延迟）
            OfflineCallback callback = nullptr;
            // 离线时的回调函数，可以使用lambda运算符在回调中传入父类，例如.callback = [this]() { this->OnOffline(); } // 捕获this
        };

        /* ====================== 3. 静态接口 ====================== */
    public:
        // 静态：周期任务，用于所有实例计数递减与离线检测
        static void updateAll();

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        explicit Daemon(const Config& cfg);
        ~Daemon();

        /* ====================== 5. 公共接口 ====================== */
    public:
        // “喂狗”——重载计数
        void reload();
        // 是否在线
        bool isOnline() const;

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        // ==== 静态全局管理 ====
        static std::array<Daemon*, MX_INS_NUM> instances_;
        static uint8_t instance_count_;

    private:
        // ==== 每个实例的成员 ====
        uint16_t reload_count_; // 计数重载值
        uint16_t temp_count_; // 当前剩余计数
        OfflineCallback callback_; // 离线回调函数
    };
}

#endif // MODULE_DAEMON_H_
