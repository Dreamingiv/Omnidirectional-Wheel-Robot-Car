//
// Created by zhangzhiwen on 25-12-21.
//

#ifndef BUZZER_ON_BOARD_H
#define BUZZER_ON_BOARD_H
#include <optional>

#include "driver_pwm.h"

namespace ega
{
	class BuzzerOnBoard
	{
	public:
		static void init();

		// 阻塞式鸣响，内部使用Hal_Delay
		static void singBlock(uint32_t Hz, uint32_t ms);

	private:
		// TODO 非阻塞式鸣响，需要结合定时器中断或者RTOS任务，没实现
		static void sing(uint32_t Hz, uint32_t ms);

		static inline std::optional<PWMInstance> pwm_;
	};
}


#endif //BUZZER_ON_BOARD_H
