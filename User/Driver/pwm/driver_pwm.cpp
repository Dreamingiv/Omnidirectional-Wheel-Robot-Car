//
// Created by 2b superman on 25-11-21.
//
#include "driver_pwm.h"

namespace ega
{
	// 静态成员初始化
	std::array<PWMInstance*, PWMInstance::MX_INS_NUM> PWMInstance::instance_list_{};
	uint8_t                                           PWMInstance::instance_count_ = 0;

	PWMInstance::PWMInstance(const Config& config)
	{
		// 1. 参数检查
		if (config.handle == nullptr)
		{
			// ERROR log...
			return;
		}

		// 2. 重复检测
		for (auto ins : instance_list_)
		{
			if (ins != nullptr && ins->handle_ == config.handle && ins->channel_ == config.channel)
			{
				// ERROR: Instance already exists
				handle_ = nullptr;
				return;
			}
		}

		// 3. 成员赋值
		this->handle_   = config.handle;
		this->channel_  = config.channel;
		this->type_     = config.type;
		this->callback_ = config.callback;

		// 4. 获取硬件参数
		// 获取当前的 ARR (Period)，用于后续比例计算
		// 注意：如果运行时动态改变了频率(修改ARR)，需要手动更新此值或调用 setRatio 时重读
		this->arr_value_ = __HAL_TIM_GET_AUTORELOAD(handle_);

		// 5. 注册到列表
		if (instance_count_ < MX_INS_NUM)
		{
			instance_list_[instance_count_++] = this;
		}

		// 6. 自动启动 (仅针对 DIRECT 模式，DMA 模式通常等待数据准备好)
		if (config.auto_start && type_ == Type::DIRECT)
		{
			start();
		}
	}

	PWMInstance::~PWMInstance()
	{
		stop();
		// 从列表中移除并前移填补空缺
		for (uint8_t i = 0; i < instance_count_; i++)
		{
			if (instance_list_[i] == this)
			{
				instance_list_[i]                   = instance_list_[instance_count_ - 1];
				instance_list_[instance_count_ - 1] = nullptr;
				--instance_count_;
				break;
			}
		}
	}

	void PWMInstance::start() const
	{
		if (!handle_)
			return;

		if (type_ == Type::DIRECT)
		{
			HAL_TIM_PWM_Start(handle_, channel_);
		}
		// DMA 模式下 start 通常由 transmitDMA 函数触发，这里也可以预开启
	}

	void PWMInstance::stop() const
	{
		if (!handle_)
			return;

		if (type_ == Type::DMA)
		{
			HAL_TIM_PWM_Stop_DMA(handle_, channel_);
		}
		else
		{
			HAL_TIM_PWM_Stop(handle_, channel_);
		}
	}

	void PWMInstance::setDuty(float ratio)
	{
		// 安全限幅
		if (ratio < 0.0f)
			ratio = 0.0f;
		if (ratio > 1.0f)
			ratio = 1.0f;

		// 计算 CCR 值
		uint32_t target_ccr = static_cast<uint32_t>(ratio * static_cast<float>(arr_value_ + 1));

		setCCR(target_ccr);
	}

	void PWMInstance::setCCR(uint32_t raw_value)
	{
		if (!handle_)
			return;

		// 直接操作寄存器，绕过 HAL 库函数调用的开销
		__HAL_TIM_SET_COMPARE(handle_, channel_, raw_value);
	}

	void PWMInstance::setFrequency(uint32_t frequency)
	{
		arr_value_ = 1000000 / frequency - 1;
		__HAL_TIM_SET_AUTORELOAD(handle_, arr_value_);
		__HAL_TIM_SET_COMPARE(handle_, channel_, arr_value_/2);
	}


	void PWMInstance::transmitDMA(uint32_t* buffer, uint16_t length)
	{
		if (!handle_ || type_ != Type::DMA)
			return;

		// 启动 DMA 传输
		// 注意：buffer 的数据类型宽度必须与 CubeMX 中 DMA 设置的 Data Width (Word/Half Word) 匹配
		HAL_TIM_PWM_Start_DMA(handle_, channel_, buffer, length);
	}

	void PWMInstance::setMode(Type type)
	{
		// 切换模式前最好先停止
		stop();
		type_ = type;
	}
}

// --- HAL 友元回调函数 ---

extern "C" {
/**
     * @brief 定时器 PWM 脉冲完成回调
     * @note  在 DMA 传输完成 (Transfer Complete) 或 开启了 IT 模式的更新事件时触发
     */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
	using namespace ega;
	for (uint8_t i = 0; i < PWMInstance::instance_count_; i++)
	{
		ega::PWMInstance* ins = PWMInstance::instance_list_[i];

		// 1. 匹配句柄
		if (ins && ins->handle_ == htim)
		{
			// 2. 匹配通道
			// HAL库的一个痛点：HAL_TIM_PWM_PulseFinishedCallback 传入的 htim
			// 只有在 ActiveChannel 被置位时才能区分通道。
			// DMA 模式下，HAL 库通常会自动处理 ActiveChannel。

			bool channel_match = false;
			if (htim->Channel == ins->channel_) // 理想情况
			{
				channel_match = true;
			}
			// 如果是 DMA 模式，HAL 内部通常会将 ActiveChannel 设置为当前正在 DMA 的通道
			else if (ins->type_ == PWMInstance::Type::DMA && htim->hdma[TIM_DMA_ID_CC1 + (ins->channel_ >> 2)] !=
				nullptr)
			{
				// 这是一个简化的推断，实际工程中如果多个通道同时开启 DMA，
				// 这里可能需要更复杂的判断逻辑，或者干脆让上层应用自己区分
				channel_match = true;
			}

			if (channel_match && ins->callback_)
			{
				ins->callback_();
			}
		}
	}
}
}
