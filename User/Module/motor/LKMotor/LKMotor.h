//
// Created by lenovo on 2025/10/31.
//

#ifndef MODULE_LKMOTOR_H_
#define MODULE_LKMOTOR_H_

#include "motor.h"

namespace ega
{
    /**
     * 目前翎控电机仅支持输入转矩闭环指令，读取状态2的所有信息。暂未做等效扭矩等适配
     */
    class LKMotor : public Motor
    {
        /* ====================== 0. Config（从基类拆出） ====================== */
    public:
        struct Config
        {
            Direction direction = Direction::NORMAL;
            FDCAN_HandleTypeDef* can_handle ;
            // 翎控电机专有字段（与旧 Motor_old::LKMotorConfig 一致）
            uint8_t motor_id = 0;       // 从1到32，必须提供
            uint8_t encoder_bits = 16;  // 编码器位数：14/15/16...
        };

        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        static constexpr size_t MAX_MOTORS = 12; // 从跃鹿抄的（主要目的可能是静态数组分配）
        static constexpr float OUTPUT_MAX = 2048.0f; //翎控电机最大输出值

        /* ====================== 2. enum ====================== */
    public:
        enum ModeCommand {
            MOTOR_MODE = 0x88,   // 使能,会响应指令
            RESET_MODE = 0x80,   // 失能
            STOP_MODE = 0x81,    // 停止，但是不清除运行状态
            READ_CMD = 0x9C,   // 读取状态
            TORQUE_CMD = 0xA1,   // 设置转矩闭环指令，
            TORQUE_ZERO_CMD = 0xA1,   // 零力矩指令，此时要求其它data全为0。同时会让电机反馈状态，等同于READ_CMD
        };

        struct LKMeasure {
            //编码器值encoder（uint16_t类型
            //14bit编码器的数值范围0~16383，15bit编码器的数值范围0~32767，16bit编码器的数值范围0~65535）。
            uint16_t encoder = 0;
            uint16_t last_encoder = 0;
            //MF、MG电机的转矩电流值iq或MS电机的输出功率值power，int16_t类型。
            //MG电机iq分辨率为(66/4096 A) / LSB；MF电机iq分辨率为(33/4096 A) / LSB。MS电机power范围-1000~1000。
            int16_t current_bit = 0;
            int8_t temperature = 0.0f;      //摄氏度 1℃/LSB
        };

        /* ====================== 3. 静态接口 ====================== */
    public:
        static void controlAll(); // 控制循环
        static void enableAll();  // 使能所有电机
        static void disableAll(); // 禁用所有电机
        static void sendCommandAll();

        static bool hasDisabledMotor();
        static bool hasOfflineMotor();

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        explicit LKMotor(const Config& config);
        ~LKMotor() override;

		/* ====================== 5. 公共接口 ====================== */
	public:
    public:
        // 发送使能失能或清除命令
        void sendLKModeCommand(ModeCommand cmd);
		void parseData(const uint8_t *data, uint8_t len);
		void sendCommand() override;

		void enable() override;
		void disable() override;

		//重设零点
		// void setZeroPosition();

		//getter
		LKMeasure &getLKMeasure() { return lk_measure_; }

		//调试信息
		void debug() override{
		    logger_printf("LKMotor Effort:%f\r\n", effort_); //用户可以自行修改想要查看的中间变量
		};
        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        static inline std::array<LKMotor *, MAX_MOTORS> motors_{};
        static inline size_t idx_ = 0;

        uint8_t motor_id_;
        const uint8_t encoder_bits_;

        LKMeasure lk_measure_{};

    private:
        // 电机掉线回调
        void offlineCallback() override;
    };
} // namespace ega

#endif // MODULE_LKMOTOR_H_
