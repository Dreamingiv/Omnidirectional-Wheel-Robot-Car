//
// Created by zhangzhiwen on 25-12-21.
//

#ifndef BUZZER_ON_BOARD_H
#define BUZZER_ON_BOARD_H
#include <optional>

#include "daemon.h"
#include "driver_pwm.h"

namespace ega
{
	class BuzzerOnBoard
	{
	public:
		static void init();

		// 阻塞式鸣响，内部使用Hal_Delay
		static void singBlock(uint32_t Hz, uint32_t ms);
	    // 非阻塞式鸣响，需要结合daemon和对应的RTOS任务
	    static void sing(uint32_t Hz, uint32_t ms);

		static void start(uint32_t Hz);

		static void stop();

	private:
		static inline std::optional<PWMInstance> pwm_;
	    static inline std::optional<Daemon> daemon_;
	};
}


#endif //BUZZER_ON_BOARD_H
