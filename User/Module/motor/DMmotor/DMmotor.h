//
// Created by An on 2025/9/29.
//

#ifndef MODULE_DMMOTOR_H_
#define MODULE_DMMOTOR_H_

#include "driver_can.h"
#include "motor.h"
#include "daemon.h"
#include <array>
#include <algorithm>  // for std::find
#include <optional>
namespace ega
{
	class DMMotor : public Motor {
		/* ====================== 1. 编译期常量 & 类型别名 ====================== */
	public:
		static constexpr size_t MAX_MOTORS = 12;//理论上同一个车的达妙电机数量没有上限，这里是从跃鹿抄的（主要目的可能是静态数组分配）
		static constexpr float KP_MIN = 0.0f;
		static constexpr float KP_MAX = 500.0f;
		static constexpr float KD_MIN = 0.0f;
		static constexpr float KD_MAX = 5.0f;

		static constexpr float SPEED_SMOOTH_COEF = 0.85f;      // 最好大于0.85
		static constexpr float TORQUE_SMOOTH_COEF = 0.9f;     // 必须大于0.9

		/* ====================== 2. 内部类型定义 ====================== */
	public:
		enum ModeCommand {
			MOTOR_MODE = 0xfc,   // 使能,会响应指令
			RESET_MODE = 0xfd,   // 停止
			ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
			CLEAR_ERROR = 0xfb // 清除电机过热错误
		};

		struct DMMeasure {
			uint8_t id{};//电机ID低8位
			uint8_t state{};//错误状态码

			//达妙原装数据
			float angle_rad = 0.0f;        //rad,大概四圈
			float speed_rads = 0.0f;        //rad/s
			//float torque = 0.0f;//基类已有
			float temperature_mos = 0.0f;      //摄氏度
			float temperature_rotor = 0.0f;    //摄氏度

			float last_angle = 0.0f; //degree
		};

		struct msg_t {
			uint16_t position_des;
			uint16_t velocity_des;
			uint16_t torque_des;
			uint16_t kp;
			uint16_t kd;
		};

		/* ====================== 3. 静态接口 ====================== */
	public:
		static void controlAll();//控制循环

		static void enableAll();//使能所有电机，建议上电后等待几秒再启用
		static void disableAll();//禁用所有电机

		static void sendCommandAll();

		/* ====================== 4. 构造 / 析构 ====================== */
	public:
		explicit DMMotor(const Config &config);
		~DMMotor() override;

		/* ====================== 5. 公共接口 ====================== */
	public:
		//直接由外部给定三个参数，在WorkMode为Other时启用
		void setMIT(float position, float velocity, float torque,float kp,float kd);

		void parseData(const uint8_t *data, uint8_t len) override;
		void sendCommand() override;

		void enable() override;
		void disable() override;

		//重设零点
		void setZeroPosition();

		//getter
		DMMeasure &getDMMeasure() { return dm_measure_; }

		//调试信息
		void debug() override{
		    logger_printf("DMMotor Effort:%f\r\n", effort_); //用户可以自行修改想要查看的中间变量
		};

		/* ====================== 6. 受保护接口 ====================== */

		/* ====================== 7. 成员变量 ====================== */
	private:
		static inline std::array<DMMotor *, MAX_MOTORS> motors_{};
		static inline size_t idx_ = 0;
	private:
	    //float effort=0.0f;

		DMMeasure dm_measure_;
		//msg_t msg_mailbox_{};

		bool use_mit_=false;

	    float p_max_abs_ =0.0f;
	    float v_max_abs_ =0.0f;
	    float t_max_abs_ =0.0f;

	    float p_des_buf_ =0.0f;
	    float v_des_buf_ =0.0f;
	    float t_des_buf_ =0.0f;
	    float kp_buf_ =0.0f;
	    float kd_buf_ =0.0f;

		//减速比
		//指外部添加的减速比,与大疆含义不同。达妙电机可以直接输出输出轴速度和位置
		float dm_reduction_radio_=1.0f;
//        float dm_torque_constant_=0.0f;
	private:
		// 电机掉线回调
		void offlineCallback() override;
		// 发送使能失能或清除命令
		void sendDMModeCommand(ModeCommand cmd);
	};
}
#endif //MODULE_DMMOTOR_H_
