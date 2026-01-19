#pragma once
#include <array>

#include "controller.h"

namespace ega
{
    class SMC:public Controller
    {
        /**
         *  @brief 滑模控制器
         *  实现单输入滑模控制，包含非线性项、误差微分滤波、死区与目标斜坡
         */
    public:
        using Input = float;
        using Output = float;
        static constexpr float EPS = 1e-6f;

    public:
        struct Config
        {
            float J = 0.0f;          // 被控对象惯量参数

            float k = 2.0f;          // 控制增益，影响响应速度和抖振
            float band = 0.5f;      // 终端阈值，小于该误差时切换控制率

            float lambda = 1.0f;     // 滑模面系数，调节误差与误差导数权重
            float alpha = 0.8f;      // 误差微分低通滤波系数 (0~1)，越小越平滑
            float epsilon = 1.0f;    // 平滑因子，符号函数替代用以抑制抖振
            float phi = 0.05f;       // 边界层宽度，决定滑模层厚度与平滑度
            float beta = 2.0f;       // 自适应系数，控制增益调整速度

            float q = 21.0f;         // 非线性滑模参数 q，定义收敛律形状
            float p = 27.0f;         // 非线性滑模参数 p，通常需大于 q

            float limit_output = 1e6f; // 输出限幅
            bool use_ramp = false;   // 是否启用目标斜坡输入
            float ramp_step = 0.0f;  // 目标斜坡步进，每周期目标值增量

            float dt = 2.0f;       // 控制周期（ms）
        };

    public:
        explicit SMC(const Config& config);
        ///单输入单输出，位置微分得到速度
        float calculate(float target, float measure);
        ///todo 输入measure的同时输入measure_dot的重载
        float calculate(float target, float measure, float measure_dot);
        void clear();

    private:
        float sat(float x) const;

    private:
        // ================= 控制参数 =================
        const float J_;


        const float k_;
        const float lambda_;
        const float lambda_fast_;
        const float alpha_;

        const float band_;
        const float epsilon_;
        const float phi_;
        const float beta_;

        const float q_;
        const float p_;

        const float limit_output_;
        const bool use_ramp_;
        const float ramp_step_;

        const float dt_;

        // ================= 状态变量 =================
        float last_target_;
        float last_target_dot_;
        float last_error_;
        float last_error_dot_;
        float last_measure_dot_;
        float last_output_; // 预留：可用于输出平滑

        // 中间变量
        float error_;
        float error_dot_;
    };
}
