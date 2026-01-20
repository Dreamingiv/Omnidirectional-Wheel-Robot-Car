//
// Created by An on 2025/9/30.
//
#ifndef MODULE_MOTOR_H_
#define MODULE_MOTOR_H_

#include <optional>
#include "daemon.h"
#include "driver_can.h"
#include "logger.h"
#include "utils.h"

namespace ega
{
    class Motor
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        static constexpr float RAD_TO_DEGREE = 57.2957795f; // 180.0f/3.1415f
        static constexpr float DEGREE_TO_RAD = 1.0f / RAD_TO_DEGREE; // 0.0174532925f
        static constexpr float EFFORT_MAX_ABS = 10.0f;//所有电机的effort值均需要被映射至-10到10的区间，实现参数量级统一
        static constexpr float PI = 3.1415926f;

        // 正反标记位，涉及数值运算，请勿改为其他数字
        enum class Direction : int8_t
        {
            NORMAL = 1,
            REVERSE = -1
        };

        // 同一单位：弧度，弧度每秒，牛米
        struct Measure
        {
            float angle_rotor = 0.0f; // 【转子】位置。根据电机不同，可能不止2pi
            float total_angle_rotor = 0.0f; // 【转子】上电后经过的完整弧度
            float total_angle = 0.0f; // 【输出轴】上电后经过的完整弧度

            float speed_rotor = 0.0f; // 【转子】速度 弧度每秒
            float speed = 0.0f; // 【输出轴】速度 弧度每秒

            float torque = 0.0f; // 等效力矩，大疆电机使用扭矩常数和减速比线性映射出等效力矩。翎控电机暂时不提供力矩
            float current = 0.0f; // 电流 A。达妙电机暂时不提供电流值。
            int32_t round = 0; // 圈数
        };

        /* ====================== 2. 静态接口（维持原结构） ====================== */
    public:
        static void controlAll();

        static void enableAll();

        static void disableAll();

        static bool hasDisabledMotor();

        static bool hasOfflineMotor();

        static void syncEnableStateAll();

        /* ====================== 3. 构造 / 析构 ====================== */
    public:
        virtual ~Motor() = default;

        /* ====================== 4. 公共接口 ====================== */
    public:
        // 设置控制输出
        virtual void setEffort(const float& effort)
        {
            effort_ = float(direction_) * utils::limit(effort, -EFFORT_MAX_ABS, EFFORT_MAX_ABS);
            is_effort_set_ = true;
        }

        virtual void unsetEffort()
        {
            effort_ = 0.0f;
            is_effort_set_ = false;
        }

        // 同步电机启用状态（派生类实现）
        virtual void syncEnableState() = 0;

        // 发送控制命令（派生类实现）
        virtual void sendCommand() = 0;

        /* 调整设置 */
        virtual void setMotorDirection(const Direction& direction)
        {
            direction_ = direction;
        };

        // 启用/禁用电机（通用）
        // 注意，这里仅仅设置enable标志位，然后通过syncEnableState()同步到电机
        virtual void enable() { is_enabled_ = true; }
        virtual void disable() { is_enabled_ = false; }

        // 调试信息
        virtual void debug()
        {
            logger_printf("Motor effort:%f\r\n", effort_); // 用户可以自行修改想要查看的中间变量
        };

        // 电机掉线回调
        virtual void offlineCallback() = 0;

        // 查询状态
        [[nodiscard]] virtual bool isEnabled() const { return is_enabled_; }
        [[nodiscard]] virtual bool isOnline() const { return is_online_; }
        [[nodiscard]] virtual bool isEffortSet() const { return is_effort_set_; }

        // 查询输出
        [[nodiscard]] virtual float getEffort() const { return effort_; }
        [[nodiscard]] virtual float getOutputMax() const { return output_max_abs_; }

        // 获取测量值
        [[nodiscard]] Measure getMeasure() const { return measure_; }

        /* ====================== 5. 受保护接口 ====================== */
    protected:
        // 映射effort到输出值，范围从-10到10映射到该电机所允许的最大输出值
        constexpr float mapEffortToOutput(const float & effort) const
        {
            return effort * output_max_abs_ / EFFORT_MAX_ABS;
        }


        /* ====================== 6. 成员变量（保持原布局） ====================== */
    protected:
        float effort_ = 0.0f; //-10到10之间的浮点数
        bool is_effort_set_ = false;

        // 电机输入值的最大范围，需要根据不同电机类型进行自动设置，然后让effort依据这个值映射完再输出。
        float output_max_abs_ = 0.0f;

        // 电机状态
        bool is_online_ = false; // 电机是否在线
        bool is_enabled_ = false; // 电机是否使能

        // 电机测量值
        Measure measure_{};

        // 电机设置
        // Type type_ = Type::OTHER;
        Direction direction_ = Direction::NORMAL; // 电机正反转标志

        // 电机内部维护实例，均使用optional延迟到派生类再注册
        std::optional<CANInstance> can_;
        std::optional<Daemon> daemon_;
    };
} // namespace ega

#endif // MODULE_MOTOR_H_
