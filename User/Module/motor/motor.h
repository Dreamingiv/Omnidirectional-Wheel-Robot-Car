//
// Created by An on 2025/9/30.
//
#ifndef MODULE_MOTOR_H_
#define MODULE_MOTOR_H_

#include <optional>
#include "controller.h"
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
        static constexpr float RAD_2_DEGREE = 57.2957795f; // 180.0f/3.1415f
        static constexpr float DEGREE_2_RAD = 1.0f / RAD_2_DEGREE;

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        enum class Type
        {
            OTHER = 0,
            // DJI电机
            GM6020,
            GM6020_CURRENT,
            M3508,
            M2006,
            // 达妙电机
            DM_MOTOR
            // 其它电机
        };

        enum class Direction : int8_t
        { // 正反标记位，涉及数值运算，请勿改为其他数字
            NORMAL = 1,
            REVERSE = -1
        };

        struct Measure
        { // 同一单位：度，度每秒，牛米
            float angle_rotor = 0.0f; // 【转子】位置。根据电机不同，可能不止360度
            float total_angle_rotor = 0.0f; // 【转子】上电后经过的完整角度
            float total_angle = 0.0f; // 【输出轴】上电后经过的完整角度

            float speed_rotor = 0.0f; // 【转子】速度 度每秒
            float speed = 0.0f; // 【输出轴】速度 度每秒

            float torque = 0.0f; // 等效力矩，大疆电机使用扭矩常数和减速比线性映射出等效力矩
            float current = 0.0f; // 电流 A。达妙电机暂时不提供电流值
            int32_t round = 0; // 圈数
        };

        // 这些Config只用于记录初始化配置，其实例应该由各电机自己生成
        struct Config
        {
            // 电机基本设置
            Type type = Type::OTHER;
            Direction direction = Direction::NORMAL;

            // 驱动和控制器设置
            CANInstance::Config can_config;

            // 各品牌电机所需专有的config
            // 大疆电机专有的初始化配置
            struct DJIMotorConfig
            {
                uint8_t motor_id = 0; // 从1到8，必须提供
                float reduction_radio = 0.0f; // 减速比。不提供时默认按照电机默认设置来。无减速箱的写1：1
            } dji_motor_config;

            // 达妙电机专有的初始化配置
            struct DMMotorConfig
            {
                // 每款达妙电机的参数不一样，使用前请一定连接到上位机确认参数，否则会造成速度或者力矩输出错误！
                float p_max_abs ;//= 12.5f; 强制用户输入
                float v_max_abs ;//= 45.0f;
                float t_max_abs ;//= 18.0f;
//                float torque_constant = 0.0f; // 转矩常数，暂时不提供
                float reduction_radio = 1.0f; // 减速比。达妙电机绝大多数能直接输出输出轴信息，默认减速比为1

                bool use_mit = false; // 仅当use_mit为true时，才使用自带的MIT模式
            } dm_motor_config;
        };

        /* ====================== 3. 静态接口 ====================== */
    public:
        // 所有电机的总控制循环
        static void controlAll();
        // 使能所有电机
        static void enableAll();
        // 禁用所有电机
        static void disableAll();
        // 查询是否存在失能电机
        static bool hasDisabledMotor();
        static bool hasOfflineMotor();

        // 工厂函数，方便上层创建电机
        static std::unique_ptr<Motor> create(const Config& config);

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        virtual ~Motor() = default;

        /* ====================== 5. 公共接口 ====================== */
    public:
        // 数据解析接口，纯虚
        virtual void parseData(const uint8_t* data, uint8_t len) = 0;

        /* 核心工作函数 */
        /**
         * @brief 设置电机输出“力量”的大小
         * @attention 根据电机类型不同，单位也不同！可能是电流，也可能是力矩等。
         */
        virtual void setEffort(const float& effort)
        {
            effort_ = float(direction_)*utils::limit(effort, -effort_max_abs_, effort_max_abs_);
            is_effort_set =true;
        }
        virtual void unsetEffort()
        {
            effort_ = 0.0f;
            is_effort_set =false;
        }

        // 发送控制命令
        virtual void sendCommand()=0; //电机应该自己实现

        /* 调整设置 */
        // 调整电机旋转方向
        virtual void setMotorDirection(const Direction &direction) { direction_ = direction; };

        // 启用/禁用电机（通用）
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
        [[nodiscard]] virtual bool isEffortSet() const { return is_effort_set; }

        // getter函数
        [[nodiscard]] Measure getMeasure() const { return measure_; }
        [[nodiscard]] virtual float getEffort() const { return effort_; }
        [[nodiscard]] virtual float getEffortMax() const { return effort_max_abs_; }



        /* ====================== 6. 受保护接口 ====================== */
    protected:
        void initParams(const Config& config)
        { // 初始化所有基类参数，在派生类构造函数里调用
            /* ===== 初始化基类参数 ===== */
            type_ = config.type;
            direction_ = config.direction;
        };

        /* ====================== 7. 成员变量 ====================== */
    protected:
        /**
         * @brief 表示电机输出“力量”的大小
         * @attention 根据电机类型不同，单位也不同，可能是电流，也可能是电压。
         */
        float effort_ = 0.0f; // 表示电机输出“力量”的大小，根据电机类型不同，单位也不同，可能是电流，也可能是电压。
        bool is_effort_set = false;
        float effort_max_abs_ = 0.0f; //effort的最大值

        bool is_online_ = false; // 电机是否在线,根据是否能接到回调决定。
        bool is_enabled_ = false; // 电机是否使能,根据用户意图决定是否使用电机。

        Measure measure_{};

        // 电机设置
        Type type_ = Type::OTHER;
        Direction direction_ = Direction::NORMAL; // 电机正反转标志

        // 电机内部维护实例，均使用optional延迟到派生类再注册
        std::optional<CANInstance> can_;
        std::optional<Daemon> daemon_;
    };
} // namespace ega

#endif // MODULE_MOTOR_H_
