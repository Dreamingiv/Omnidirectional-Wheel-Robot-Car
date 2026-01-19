#include "SMC.h"
#include <cmath>
#include "utils.h"

namespace ega
{
    SMC::SMC(const Config& config) :
        J_(config.J),
        k_(config.k),
        lambda_(config.lambda),
        lambda_fast_(config.lambda * config.q / (config.p == 0.0f ? 1.0f : config.p)),
        alpha_(utils::limit(config.alpha, 0.0f, 1.0f)),
        band_(config.band),
        epsilon_(config.epsilon),
        phi_(config.phi),
        beta_(config.beta),
        q_(config.q),
        p_(config.p),
        limit_output_(config.limit_output),
        dt_(config.dt),
        use_ramp_(config.use_ramp),
        ramp_step_(config.ramp_step)
    {
        error_ = 0.0f;
        error_dot_ = 0.0f;

        last_target_ = 0.0f;
        last_error_ = 0.0f;
        last_error_dot_ = 0.0f;
        last_target_dot_ = 0.0f;
        // last_measure_ = 0.0f;
        last_measure_dot_ = 0.0f;
        last_output_ = 0.0f;
    }

    float SMC::calculate(float target, float measure)
    {
        // ================= 1. Ramp 处理 =================
        if (use_ramp_)
        {
            target = utils::limit(
                target,
                last_target_ - ramp_step_ * dt_,
                last_target_ + ramp_step_ * dt_
                );
        }

        if (dt_ == 0.0f)
            return 0.0f;

        // ================= 2. 误差计算 =================
        float raw_error = measure - target;
        if (std::fabs(raw_error) < EPS)
        {
            raw_error = std::copysignf(EPS, raw_error == 0.0f ? 1.0f : raw_error);
        }
        error_ = raw_error; //储存中间变量

        float raw_error_dot = (error_ - last_error_) / dt_;
        float filtered_error_dot = (1.0f - alpha_) * last_error_dot_ + alpha_ * raw_error_dot; //一阶低通滤波
        if (std::fabs(filtered_error_dot) < EPS)
        {
            filtered_error_dot = std::copysignf(EPS, filtered_error_dot);
        }
        error_dot_ = filtered_error_dot; //储存中间变量

        // ================= 3. 有效 epsilon =================
        const float epsilon_effective = epsilon_ / (1.0f + beta_ * std::fabs(error_));

        // ================= 4. 非线性误差 =================
        const float exponent = (p_ == 0.0f) ? q_ : (q_ / p_);
        const float error_abs = std::fabs(error_);
        const float error_nonlinear = std::copysignf(std::pow(error_abs, exponent), error_);

        // ================= 5. 目标速度与加速度 =================
        const float target_dot = (target - last_target_) / dt_;
        const float target_dot_dot = (target_dot - last_target_dot_) / dt_;

        // ================= 6. 控制律计算 =================
        float s = 0.0f;
        float output = 0.0f;

        if (std::fabs(error_) <= band_)
        {
            s = error_dot_ + lambda_ * error_;
            output = J_ * (target_dot_dot - lambda_ * error_dot_ - k_ * s - epsilon_effective * sat(s));
        }
        else
        {
            float error_safe = std::fabs(error_); //分母
            if (error_safe < EPS)
                error_safe = EPS;
            s = error_dot_ + lambda_ * error_nonlinear;
            output = J_ * (target_dot_dot - lambda_fast_ * error_nonlinear * error_dot_ / error_safe - k_ * s -
                epsilon_effective * sat(s));
        }

        if (std::isnan(output) || std::isinf(output))
        {
            output = 0.0f;
        }

        // ================= 8. 输出限幅 =================
        output = utils::limit(output, -limit_output_, limit_output_);


        // ================= 7. 更新历史 =================
        last_error_dot_ = error_dot_;
        last_error_ = error_;
        last_output_ = output;
        last_target_ = target;
        last_target_dot_ = target_dot;
        // last_measure_ = measure;
        // last_measure_dot_ = measure_dot;

        return output;
    }

    float SMC::calculate(float target, float measure, float measure_dot)
    {
        //todo 暂时还不能用
        return 0.0f;

        // // ================= 1. Ramp 处理 =================
        // if (use_ramp_)
        // {
        //     target = utils::limit(
        //         target,
        //         last_target_ - ramp_step_ * dt_,
        //         last_target_ + ramp_step_ * dt_
        //         );
        // }
        //
        // if (dt_ == 0.0f)
        //     return 0.0f;
        //
        // // ================= 2. 误差计算（用外部提供的 measure_dot） =================
        // // 注意：你当前定义的误差是 measure - target（不是 target - measure）
        // float raw_error = measure - target;
        // if (std::fabsf(raw_error) < EPS)
        // {
        //     raw_error = std::copysignf(EPS, raw_error == 0.0f ? 1.0f : raw_error);
        // }
        // error_ = raw_error;
        //
        // // 目标速度（用于把“测量速度”转换成“误差速度”）
        // const float target_dot = (target - last_target_) / dt_;
        //
        // // 误差导数 = d(measure-target)/dt = measure_dot - target_dot
        // // float raw_error_dot = measure_dot - target_dot;
        // // error_dot_ = raw_error_dot;
        //
        // // 低通滤波测量速度（推荐）
        // float filtered_measure_dot = (1 - alpha_) * last_measure_dot_ + alpha_ * measure_dot;
        // if (std::fabsf(filtered_measure_dot) < EPS)
        // {
        //     filtered_measure_dot = 0.0f;
        // }
        // error_dot_ = filtered_measure_dot; // 微分先行：只用测量导数
        // last_measure_dot_ = filtered_measure_dot;
        //
        // // ================= 3. 有效 epsilon =================
        // const float epsilon_effective = epsilon_ / (1.0f + beta_ * std::fabsf(error_));
        //
        // // ================= 4. 非线性误差 =================
        // const float exponent = (p_ == 0.0f) ? q_ : (q_ / p_);
        // const float error_abs = std::fabsf(error_);
        // const float error_nonlinear = std::copysignf(std::powf(error_abs, exponent), error_);
        //
        // // ================= 5. 目标加速度 =================
        // const float target_dot_dot = (target_dot - last_target_dot_) / dt_;
        //
        // // ================= 6. 控制律计算 =================
        // float s = 0.0f;
        // float output = 0.0f;
        //
        // if (std::fabsf(error_) <= band_)
        // {
        //     s = error_dot_ + lambda_ * error_;
        //     output = J_ * (target_dot_dot - lambda_ * error_dot_ - k_ * s - epsilon_effective * sat(s));
        // }
        // else
        // {
        //     float error_safe = std::fabsf(error_);
        //     if (error_safe < EPS)
        //         error_safe = EPS;
        //
        //     s = error_dot_ + lambda_ * error_nonlinear;
        //     output = J_ * (target_dot_dot
        //         - lambda_fast_ * error_nonlinear * error_dot_ / error_safe
        //         - k_ * s
        //         - epsilon_effective * sat(s));
        // }
        //
        // if (std::isnan(output) || std::isinf(output))
        // {
        //     output = 0.0f;
        // }
        // // ================= 8. 输出限幅 =================
        // output = utils::limit(output, -limit_output_, limit_output_);
        //
        // // ================= 7. 更新历史 =================
        // last_error_dot_ = error_dot_;
        // last_error_ = error_;
        // last_output_ = output;
        // last_target_ = target;
        // last_target_dot_ = target_dot;
        //
        // return output;

    }

    void SMC::clear()
    {
        error_ = 0.0f;
        error_dot_ = 0.0f;
        last_output_ = 0.0f;
        last_error_ = 0.0f;
        last_error_dot_ = 0.0f;
        last_target_ = 0.0f;
        last_target_dot_ = 0.0f;
        last_measure_dot_ = 0.0f;
    }

    float SMC::sat(const float x) const
    {
        if (phi_ == 0.0f)
            return (x > 0 ? 1.0f : (x < 0 ? -1.0f : 0.0f));
        return std::tanh(x / phi_);
    }

}
