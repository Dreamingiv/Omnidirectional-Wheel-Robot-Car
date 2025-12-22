//
// Created by An on 2025/9/30.
//

#ifndef MODULE_CONTROLLER_PIDCONTROLLER_H_
#define MODULE_CONTROLLER_PIDCONTROLLER_H_

#include "controller.h"

namespace ega
{
    class PID : public Controller
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */

        /* ====================== 2. 内部类型定义 ====================== */

        /* ====================== 3. 静态接口 ====================== */

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        explicit PID(const Config::PIDConfig& config);

        /* ====================== 5. 公共接口 ====================== */
    public:
        float calculate(float target, float measure, float forward) override;
        void clear() override;

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        float kp_ = 0.0f;
        float ki_ = 0.0f;
        float kd_ = 0.0f;

        float limit_output_ = 0.0f;
        float limit_integral_ = 0.0f;
        float limit_error_ = 0.0f;
        float limit_diff_ = 0.0f;

        float dead_band_ = 0.0f;

        float ramp_step_ = 0.0f;

        bool use_ramp_ = false;

        float measure_ = 0.0f;
        float last_measure_ = 0.0f;

        float error_ = 0.0f;
        float last_error_ = 0.0f;

        float pout_ = 0.0f;
        float iout_ = 0.0f;
        float dout_ = 0.0f;

        float output_ = 0.0f;
        float last_output_ = 0.0f;

        float target_ = 0.0f;
        float last_target_ = 0.0f;

        float dt_ = 2.0f;
    };
}


#endif //MODULE_CONTROLLER_PIDCONTROLLER_H_
