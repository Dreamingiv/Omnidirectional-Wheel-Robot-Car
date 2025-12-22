//
// Created by An on 2025/9/30.
//

#ifndef MODULE_CONTROLLER_CONTROLLER_H_
#define MODULE_CONTROLLER_CONTROLLER_H_
#include <memory>

namespace ega
{
    class Controller
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        /* 控制器类型枚举 */
        enum class Type
        {
            NONE = 0,
            PID,
            SMC
            //...待添加
        };

        struct Config
        {
            Type type = Type::NONE;

            struct PIDConfig
            {
                float kp = 0;
                float ki = 0;
                float kd = 0;

                float limit_output = 1e6f;
                float limit_integral = 1e6f;
                float limit_error = 1e6f;
                float limit_diff = 1e6f;

                float dead_band = 0.0f;

                //			enum class Mode {
                //				NORMAL = 0,
                //				RAMP,
                //			} mode = Mode::NORMAL;
                bool use_ramp = false;
                float ramp_step = 0;

                float dt = 2; //ms
            } pid_config;

            struct SMCConfig
            {
                // === 必须由用户指定 ===
                float J = 0.0f; // 被控对象惯量参数，用于系统动态计算
                float dt = 0.005f; // 控制周期（秒），需与任务周期一致

                // === 常用调参项 ===
                float k = 2.0f; // 控制增益系数，影响响应速度与抖振强度
                float lambda = 1.0f; // 滑模面系数，调节误差与误差导数权重
                float dead_band = 0.05f; // 死区阈值，小于此误差时输出归零
                float alpha = 1.0f; // 非线性项比例系数，影响滑模面形状
                float epsilon = 1.0f; // 平滑因子，符号函数替代用以抑制抖振
                float phi = 0.05f; // 边界层宽度，决定滑模层厚度与平滑度
                float beta = 2.0f; // 自适应系数，控制增益调整速度

                // === 次要或特性参数 ===
                bool use_ramp = false; // 是否启用目标斜坡输入
                float ramp_step = 0.0f; // 目标斜坡步进，每周期目标值增量
                float q = 21.0f; // 非线性滑模参数 q，定义收敛律形状
                float p = 27.0f; // 非线性滑模参数 p，通常需大于 q
            } smc_config;
        };

        /* ====================== 3. 静态接口 ====================== */
    public:
        static std::unique_ptr<Controller> create(Config config);
        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        virtual ~Controller() = default;

        /* ====================== 5. 公共接口 ====================== */
        //纯虚函数
        virtual float calculate(float target, float measure, float forward) = 0;
        virtual void clear() = 0;

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:

    };
}


#endif //MODULE_CONTROLLER_CONTROLLER_H_
