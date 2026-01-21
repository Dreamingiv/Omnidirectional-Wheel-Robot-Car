//
// Created by zhangzhiwen on 25-12-21.
//

#include "buzzer_on_board.h"

namespace ega
{
	void BuzzerOnBoard::init()
	{
		pwm_.emplace(
			PWMInstance::Config
			{
				.handle = &htim4,
				.channel = TIM_CHANNEL_3,
				.type = PWMInstance::Type::DIRECT,
				.auto_start = false,
				.callback = nullptr
			}
		);
		pwm_->setFrequency(4000);
		// singBlock(4000, 200);
	}

	void BuzzerOnBoard::sing(uint32_t Hz, uint32_t ms)
	{
		pwm_->setFrequency(Hz);
		pwm_->start();
		// HAL_Delay(ms);
		if (daemon_.has_value())
		{
			daemon_.reset();
		}
		Daemon::Config daemon_config = {
			.reload_time_ms = static_cast<uint16_t>(ms),
			.init_time_ms = static_cast<uint16_t>(ms),
			.callback = []()
			{
				pwm_->stop();
			},
			.trig_mode = Daemon::TrigMode::Single,
		};
		daemon_.emplace(daemon_config);
	}


	void BuzzerOnBoard::singBlock(uint32_t Hz, uint32_t ms)
	{
		pwm_->setFrequency(Hz);
		pwm_->start();
		HAL_Delay(ms);
		pwm_->stop();
	}

	void BuzzerOnBoard::start(uint32_t Hz)
	{
		pwm_->setFrequency(Hz);
		pwm_->start();
	}

	void BuzzerOnBoard::stop()
	{
		pwm_->stop();
	}
}
