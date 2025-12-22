//
// Created by An on 2025/11/1.
//

#ifndef MODULE_CONTROLLER_SMCCONTROLLER_H_
#define MODULE_CONTROLLER_SMCCONTROLLER_H_

#include "controller.h"

namespace ega
{
    class SMCInstance : public Controller
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */

        /* ====================== 2. 内部类型定义 ====================== */

        /* ====================== 3. 静态接口 ====================== */

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        explicit SMCInstance(const Config::SMCConfig& config);

        /* ====================== 5. 公共接口 ====================== */
    public:
        float sgn(float x);
        float sat(float x);
        float calculate(float target, float measure, float forward) override;
        void clear() override;

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        // 控制参数
        float J_;
        float epsilon_;
        float k_;
        float lambda_ = 1.0f;
        float lambda_fast_ = 1.0f;
        float alpha_;
        float q_;
        float p_;
        float phi_ = 0.05;
        float beta_ = 2.0;
        float alpha_dot = 0.5;
        float dead_band_ = 0.01f;

        float eps_eff_;
        float e_qp_;
        float s_;
        float measure_;
        float error_;
        float e_dot;
        float e_dot_last;
        float target_;
        float target_last_;
        float omega_target_dot_;
        float omega_target_dot_last_;
        float omega_target_dot_dot_;
        float output_;
        float last_error_;
        float last_output_;
        float dt_ = 0.005;

        bool use_ramp_ = false;
        float ramp_step_ = 0.0f;
    };
}

#endif //MODULE_CONTROLLER_SMCCONTROLLER_H_
