#ifndef MODULE_LOGGER_H_
#define MODULE_LOGGER_H_
#include "driver_usart.h"
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <optional>

#define ERROR_LEVEL  0
#define WARN_LEVEL    1
#define INFO_LEVEL    2
#define DEBUG_LEVEL  3
#define TRACE_LEVEL    4

/**
 * 在此修改LOG等级
 * 不定义的话默认不输出log，所有相关代码会编译器被优化掉
 * 也可以在CMakelists.txt中的add_definitions添加-DLOG_LEVEL=XXX_LEVEL，但是要注意和文件不要冲突
 */
//#define LOG_LEVEL INFO_LEVEL

/**
 * @brief 日志等级
 * 初始化等级以上的不会输出，例如如果设置等级为INFO，则DEBUG和TRACE不会输出
 */

#ifdef LOG_LEVEL

#if LOG_LEVEL != ERROR_LEVEL && \
		LOG_LEVEL != WARN_LEVEL && \
		LOG_LEVEL != INFO_LEVEL && \
		LOG_LEVEL != DEBUG_LEVEL && \
		LOG_LEVEL != TRACE_LEVEL
#error "Invalid LOG_LEVEL defined! Must be one of ERROR_LEVEL, WARN_LEVEL, INFO_LEVEL, DEBUG_LEVEL, TRACE_LEVEL"
#endif

#define LOG_ERROR(fmt, ...) \
		do { if (static_cast<int>(ERROR_LEVEL) <= static_cast<int>(LOG_LEVEL)) \
				Logger::Log("[ERROR] " fmt, ##__VA_ARGS__); } while(0)

#define LOG_WARN(fmt, ...) \
		do { if (static_cast<int>(WARN_LEVEL) <= static_cast<int>(LOG_LEVEL)) \
				Logger::Log("[WARN] " fmt, ##__VA_ARGS__); } while(0)

#define LOG_INFO(fmt, ...) \
		do { if (static_cast<int>(INFO_LEVEL) <= static_cast<int>(LOG_LEVEL)) \
				Logger::Log("[INFO] " fmt, ##__VA_ARGS__); } while(0)

#define LOG_DEBUG(fmt, ...) \
		do { if (static_cast<int>(DEBUG_LEVEL) <= static_cast<int>(LOG_LEVEL)) \
				Logger::Log("[DEBUG] " fmt, ##__VA_ARGS__); } while(0)

#define LOG_TRACE(fmt, ...) \
		do { if (static_cast<int>(TRACE_LEVEL) <= static_cast<int>(LOG_LEVEL)) \
				Logger::Log("[TRACE] " fmt, ##__VA_ARGS__); } while(0)

#else

#define LOG_ERROR(fmt, ...) ((void)0)
#define LOG_WARN(fmt, ...)  ((void)0)
#define LOG_INFO(fmt, ...)  ((void)0)
#define LOG_DEBUG(fmt, ...) ((void)0)
#define LOG_TRACE(fmt, ...) ((void)0)

#endif
namespace ega
{
    /**
     * @brief Logger 单例类，用于输出调试信息
     *
     * 该类提供线程安全的日志输出功能，支持格式化字符串和 DMA/中断方式的串口传输。
     * 使用单例模式确保全局只有一个日志实例。
     */
    class Logger
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        struct Config
        {
            UARTInstance::Config uart_config{
                .handle = &huart6,
                .tx_type = ega::UARTInstance::DMA, ///< 使用 DMA 发送（可按需改为 IT）
                .rx_type = ega::UARTInstance::IT_IDLE, ///< 中断接收
                .rx_size = 64, ///< Logger 默认只发不收，如有必要后期可以拓展
                .use_fifo = true, ///< 使用 FIFO 缓冲
                .queue_mx_size = 4, ///< 队列容量，最多可连续调用4次
            };
        };

        /* ====================== 3. 静态接口 ====================== */
    public:
        /**
         * @brief 获取 Logger 单例实例
         * @return Logger& Logger 单例引用
         */
        static Logger& getInstance();

        static void init(const Config& cfg = {
            //默认配置
            .uart_config = {
                .handle = &huart6,
                .tx_type = ega::UARTInstance::DMA, ///< 使用 DMA 发送（可按需改为 IT）
                .rx_type = ega::UARTInstance::IT_IDLE, ///< 中断接收
                .rx_size = 64, ///< Logger 默认只发不收，如有必要后期可以拓展
                .use_fifo = true, ///< 使用 FIFO 缓冲
                .queue_mx_size = 4, ///< 队列容量，最多可连续调用4次
            }
        });

        /**
         * @brief 格式化输出调试信息
         * @param fmt 格式字符串
         * @param ... 可变参数列表
         */
        static void printf(const char* fmt, ...);

        /**
         * @brief 输出日志
         */
#ifdef LOG_LEVEL
		static void Log(const char *fmt, ...);
		void VLog(const char *fmt, va_list args);
#endif
        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        // 禁止拷贝和赋值
        Logger(const Logger&) = delete; ///< 禁用拷贝构造函数
        Logger& operator=(const Logger&) = delete; ///< 禁用赋值运算符
    private:
        /**
         * @brief 默认构造函数
         */
        Logger() = default;

        /**
         * @brief 默认析构函数
         */
        ~Logger() = default;

        /* ====================== 5. 公共接口 ====================== */
    public:
        /**
         * @brief va_list 版本的格式化输出，避免二次 vsnprintf
         * @param fmt 格式字符串
         * @param args 可变参数列表
         */
        void vPrintf(const char* fmt, va_list args);

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        std::optional<UARTInstance> uart_; ///< 内部维护的 UART 实例，延迟构造机制，防止双重构造
        bool inited_ = false; ///< 标识 Logger 是否已初始化
    };
}

/**
 * @brief 全局内联函数，便于快速调用调试打印
 * @param fmt 格式字符串
 * @param ... 可变参数列表
 */
inline void logger_printf(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    ega::Logger::getInstance().vPrintf(fmt, args);
    va_end(args);
}
#endif	// MODULE_LOGGER_H_
