/**
 * @file logger.cpp
 * @brief Logger 类的实现文件
 */

#include "logger.h"

namespace ega
{
    Logger& Logger::getInstance()
    {
        static Logger instance;
        return instance;
    }

    void Logger::init(const Logger::Config& cfg)
    {
        if (getInstance().inited_)
        {
            return;
        }

        getInstance().uart_.emplace(cfg.uart_config);
        getInstance().inited_ = true;
    }

    void Logger::printf(const char* fmt, ...)
    {
        if (!getInstance().inited_)
        {
            //Logger 未初始化，忽略输出
            return;
        }

        char buffer[256]; // 临时缓冲区，可根据需求调整
        va_list args;
        va_start(args, fmt);
        int len = vsnprintf((char*)buffer, sizeof(buffer), fmt, args);
        va_end(args);

        if (len > 0)
        {
            if (len >= static_cast<int>(sizeof(buffer)))
            {
                len = sizeof(buffer) - 1; // 避免溢出
            }
            getInstance().uart_->send((uint8_t*)buffer, len);
        }
    }

    void Logger::vPrintf(const char* fmt, va_list args)
    {
        if (!inited_)
            return;

        char buffer[256];
        int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
        if (len > 0)
        {
            if (len >= static_cast<int>(sizeof(buffer)))
            {
                len = sizeof(buffer) - 1;
            }
            uart_->sendCopy((uint8_t*)buffer, len);
        }
    }

#ifdef LOG_LEVEL
void Logger::Log(const char *fmt, ...) {
	if (!GetInstance().inited_) return;

	char buffer[256];
	va_list args;
	va_start(args, fmt);
	int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	if (len > 0) {
		if (len >= static_cast<int>(sizeof(buffer))) {
			len = sizeof(buffer) - 1;
		}
		GetInstance().uart_->Send((uint8_t*)buffer, len);
	}
}

void Logger::VLog(const char* fmt, va_list args) {
	if (!GetInstance().inited_) return;

	char buffer[256];
	int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
	if (len > 0) {
		if (len >= static_cast<int>(sizeof(buffer))) {
			len = sizeof(buffer) - 1;
		}
		uart_->Send((uint8_t*)buffer, len);
	}
}
#endif
}
