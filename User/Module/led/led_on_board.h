//
// Author: Breezeee
// Date: 25-5-28
//

#ifndef MODULE_LED_ON_BOARD_H_
#define MODULE_LED_ON_BOARD_H_

#include <optional>
#include "driver_pwm.h"

namespace ega
{
	class LEDOnBoard
	{
		/* ====================== 1. 编译期常量 & 类型别名 ====================== */
	public:

		/* ====================== 2. 内部类型定义 ====================== */
	public:
		struct RGB
		{
			uint8_t r;
			uint8_t g;
			uint8_t b;
		};

		struct HSV
		{
			float h;
			float s;
			float v;
		};

		struct Config
		{
			PWMInstance::Config pwm_config_r;
			PWMInstance::Config pwm_config_g;
			PWMInstance::Config pwm_config_b;
		};

		/* ====================== 3. 静态接口 ====================== */
	public:
		static LEDOnBoard& getInstance();

		static void loop();
		static void init(
			const Config& config = {
				.pwm_config_r = {
					.handle = &htim5,
					.channel = TIM_CHANNEL_3,
					.type = PWMInstance::DIRECT,
					.auto_start = true,
					.callback = nullptr
				},
				.pwm_config_g = {
					.handle = &htim5,
					.channel = TIM_CHANNEL_2,
					.type = PWMInstance::DIRECT,
					.auto_start = true,
					.callback = nullptr
				},
				.pwm_config_b = {
					.handle = &htim5,
					.channel = TIM_CHANNEL_1,
					.type = PWMInstance::DIRECT,
					.auto_start = true,
					.callback = nullptr
				}
			}
		);
		static void setColorRGB(uint8_t r, uint8_t g, uint8_t b);
		static void setColorHSV(float h, float s, float v);

		/* ====================== 4. 构造 / 析构 ====================== */
	public:
		LEDOnBoard(const LEDOnBoard&)            = delete;
		LEDOnBoard& operator=(const LEDOnBoard&) = delete;

	private:
		LEDOnBoard()  = default;
		~LEDOnBoard() = default;

		/* ====================== 5. 公共接口 ====================== */

		/* ====================== 6. 受保护接口 ====================== */

		/* ====================== 7. 成员变量 ====================== */
	private:
		void control();
		void HSVtoRGB();

		bool inited_{false};

		std::optional<PWMInstance> pwm_r;
		std::optional<PWMInstance> pwm_g;
		std::optional<PWMInstance> pwm_b;

		RGB rgb_{0};
		HSV hsv_{0};
	};
}
#endif // MODULE_LED_ON_BOARD_H_
