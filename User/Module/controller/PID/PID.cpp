#include "PID.h"

#include <cmath>

/**********************************************************************
 *                        以下内容为函数具体实现                          *
 **********************************************************************/

namespace ega
{

    PID::PID(const Config& config) :
        kp_(config.kp),
        ki_(config.ki),
        kd_(config.kd),
        limit_output_(config.limit_output),
        limit_integral_(config.limit_integral),
        limit_error_(config.limit_error),
        limit_diff_(config.limit_diff),
        dead_band_(config.dead_band),
        ramp_step_(config.ramp_step),
        use_ramp_(config.use_ramp),
        dt_(config.dt)
    {
    }

    /**
     * @brief CRTP 要求的 PID 实现
     *
     * @param target   目标值
     * @param measure  输入，一般为电机测量值
     */
    float PID::calculate(float target, float measure)
    {

        // === 1. 目标斜坡（ramp） ===
        if (use_ramp_)
        {
            target = utils::limit(
                target,
                last_target_ - ramp_step_ * dt_,
                last_target_ + ramp_step_ * dt_
            );
        }

        // === 2. 误差计算 ===
        float error = target - measure;

        error = utils::limit(error, -limit_error_, limit_error_);

        float output = 0.0f;

        if (std::abs(error) > dead_band_)
        {
            float p_out = kp_ * error;

            iout_ += ki_ * error * dt_;
            iout_ = utils::limit(iout_, -limit_integral_, limit_integral_);

            float dout = kd_ * (last_measure_ - measure) / dt_;
            dout = utils::limit(dout, -limit_diff_, limit_diff_);

            // 合成输出：
            output =p_out + iout_ + dout;
        }

        // === 3. 输出限幅 ===
        output = utils::limit(output, -limit_output_, limit_output_);

        // === 4. 状态更新 ===
        last_error_ = error;
        last_measure_ = measure;
        last_output_ = output;
        last_target_ = target;

        // === 5. 返回 Output ===
        return output;

    }

    /**
     * @brief 清空 PID 内部状态
     */
    void PID::clear()
    {
        last_output_ = 0.0f;
        iout_ = 0.0f;

        last_error_ = 0.0f;

        last_measure_ = 0.0f;

        last_target_ = 0.0f;
    }

} // namespace ega
