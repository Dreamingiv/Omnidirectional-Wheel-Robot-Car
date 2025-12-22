//
// Created by 2b superman on 25-11-21.
//
#ifndef DRIVER_PWM_H
#define DRIVER_PWM_H

// HAL
#include "tim.h"
// STL
#include <functional>
#include <array>

namespace ega
{
	/**
	* @brief PWM 硬件驱动类
	* @details 仅负责对 TIM PWM 功能的底层封装，不包含业务逻辑（如舵机角度计算）
	* @note    周期的计算目前依赖于CubeMX中的设置，不支持在代码里软件修改。TIM1和TIM2主频是240MHz
	* @attention 达妙开发板的PWM接口旁边的电源是可控电源，需要把PC15设置为HIGH。考虑到舵机不常用，默认关闭。
	*/
	class PWMInstance
	{
		/* ====================== 1. 编译期常量 & 类型别名 ====================== */
	public:
		// 声明友元函数以处理 HAL 中断/DMA 回调
		friend void             ::HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim);
		static constexpr size_t MX_INS_NUM = 8; ///< 最大支持的 PWM 实例数
		using Callback                     = std::function<void()>;

		/* ====================== 2. 内部类型定义 ====================== */
	public:
		/**
		* @brief PWM 工作类型
		*/
		enum class Type
		{
			DIRECT, ///< 直接模式：CPU 直接修改 CCR 寄存器 (最常用，低延迟)
			DMA,    ///< DMA 模式：通过 DMA 搬运内存数据到 CCR (用于生成特定时序波形)
		};

		/**
		* @brief 初始化配置结构体
		*/
		struct Config
		{
			TIM_HandleTypeDef* handle     = nullptr;
			uint32_t           channel    = 0;
			Type               type       = Type::DIRECT; ///< 默认使用直接模式
			bool               auto_start = true;         ///< 初始化后是否自动开启 PWM 输出
			Callback           callback   = nullptr;      ///< DMA 传输完成或 IT 更新中断回调
		};

		/* ====================== 3. 静态接口 ====================== */

		/* ====================== 4. 构造 / 析构 ====================== */
	public:
		explicit PWMInstance(const Config& config);
		~PWMInstance();

		// 禁止拷贝/移动
		PWMInstance(const PWMInstance&)            = delete;
		PWMInstance& operator=(const PWMInstance&) = delete;

		/* ====================== 5. 公共接口 ====================== */
	public:
		/**
		* @brief 启动 PWM 输出
		*/
		void start() const;

		/**
		* @brief 停止 PWM 输出
		*/
		void stop() const;

		/**
		* @brief [DIRECT 模式常用] 设置占空比比例
		* @param ratio 0.0f ~ 1.0f (0% ~ 100%)
		* @note 内部自动计算 CCR 值： CCR = ARR * ratio
		*/
		void setDuty(float ratio);

		/**
		* @brief [DIRECT 模式高级] 直接设置比较寄存器 (CCR)
		* @param raw_value 0 ~ ARR
		* @note 比 setDutyRatio 少一步浮点运算，效率更高，适合高频控制环路
		*/
		void setCCR(uint32_t raw_value);

		/**
		* @brief [DIRECT 模式常用] 设置PWM频率，用于蜂鸣器等。
		* @note 使用时要求设置CubeMX中的定时器预分频值使得 Fre/(PSC+1) = 1MHz, 否则频率会不准。（注意时钟树中不是所有定时器都有168MHz）。调用后会默认将占空比设为50%，需要的话再调用setDuty更改
		* @param frequency 期望设置的频率(Hz)，只提供整数的精度
		*/
		void setFrequency(uint32_t frequency);

		/**
		* @brief [DMA 模式专用] 发送一段波形数据
		* @details 启动 DMA 传输，将 buffer 中的数据依次送入 CCR 寄存器
		* @param buffer 存放 CCR 值的数组指针 (注意数据宽度需与 DMA 配置一致，通常为 uint16_t 或 uint32_t)
		* @param length 数据长度
		*/
		void transmitDMA(uint32_t* buffer, uint16_t length);

		/**
		* @brief 修改工作模式 (需停止后设置)
		*/
		void setMode(Type type);

		/**
		* @brief 获取当前定时器的自动重装载值 (ARR)
		* @return ARR value
		*/
		[[nodiscard]] uint32_t getARR() const { return arr_value_; }

		/* ====================== 6. 受保护接口 ====================== */

		/* ====================== 7. 成员变量 ====================== */
	private:
		// 静态资源管理
		static std::array<PWMInstance*, MX_INS_NUM> instance_list_;
		static uint8_t                              instance_count_;

		// 实例私有变量
		TIM_HandleTypeDef* handle_;
		uint32_t           channel_;
		Type               type_;
		Callback           callback_;

		// 缓存 ARR 值，避免每次设置占空比都读寄存器，提高效率
		uint32_t arr_value_;
	};
}
#endif //DRIVER_PWM_H
