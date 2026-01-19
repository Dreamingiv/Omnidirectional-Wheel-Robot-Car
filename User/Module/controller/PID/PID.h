//
// Created by An on 2025/12/31.
//

#pragma once

#include "controller.h"
#include "logger.h"
#include "utils.h"

namespace ega
{
    class PID: public Controller
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        using Input = float;
        using Output = float;
        // static constexpr float EPS = 1e-6f;
        /* ====================== 2. 内部类型定义 ====================== */
    public:
        struct Config
        {
            float kp = 0;
            float ki = 0;
            float kd = 0;

            float limit_output = 1e6f;
            float limit_integral = 1e6f;
            float limit_error = 1e6f;
            float limit_diff = 1e6f;

            float dead_band = 0.0f;

            bool use_ramp = false;
            float ramp_step = 0;

            float dt = 2.0f; //ms
        };

        /* ====================== 3. 静态接口 ====================== */

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        explicit PID(const Config& config);

        /* ====================== 5. 实现接口 ====================== */
    public:
        float calculate(float target, float measure);


        void clear();

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        float kp_;
        float ki_;
        float kd_;

        float limit_output_;
        float limit_integral_;
        float limit_error_;
        float limit_diff_;

        float dead_band_;

        float ramp_step_;
        bool use_ramp_;

        float last_measure_ = 0.0f;
        float last_error_ = 0.0f;
        float iout_ = 0.0f;
        float last_output_ = 0.0f;
        float last_target_ = 0.0f;

        float dt_ = 2.0f;//ms
    };

};

