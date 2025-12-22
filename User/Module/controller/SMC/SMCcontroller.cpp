#include "SMCcontroller.h"
#include <cmath>
#include "logger.h"

namespace ega
{
    SMI_SISO::SMI_SISO(const Config::SMCConfig& config) :
        J_(config.J),
        lambda_(config.lambda),
        alpha_(config.alpha),
        q_(config.q),
        p_(config.p),
        k_(config.k),
        epsilon_(config.epsilon),
        dead_band_(config.dead_band),
        phi_(config.phi),
        beta_(config.beta),
        dt_(config.dt)
    {
        measure_ = 0.0f;
        target_ = 0.0f;
        target_last_ = 0.0f;
        error_ = 0.0f;
        last_error_ = 0.0f;
        e_dot = 0.0f;
        e_dot_last = 0.0f;
        omega_target_dot_ = 0.0f;
        omega_target_dot_last_ = 0.0f;
        omega_target_dot_dot_ = 0.0f;
        s_ = 0.0f;
        output_ = 0.0f;
        last_output_ = 0.0f;
        lambda_fast_ = lambda_;
        alpha_dot = alpha_;
    }

    inline float safe_powf(float base, float exp)
    {
        // 当 base 非负时正常 powf；若 base 为 0 且 exp <= 0，避免 nan/inf
        if (base <= 0.0f)
        {
            if (exp <= 0.0f)
                return 0.0f;
            return powf(base, exp); // powf(0,positive) == 0
        }
        return powf(base, exp);
    }

    float SMI_SISO::sgn(float x)
    {
        if (x > 0.0f)
            return 1.0f;
        if (x < 0.0f)
            return -1.0f;
        return 0.0f;
    }

    float SMI_SISO::sat(float x)
    {
        // 防止 phi_ 为 0
        if (phi_ == 0.0f)
            return (x > 0 ? 1.0f : (x < 0 ? -1.0f : 0.0f));
        return tanhf(x / phi_);
    }

    float SMI_SISO::calculate(float target, float measure, float forward)
    {
        // 1) 基本防护：dt_ 不能为 0
        if (dt_ == 0.0f)
        {
//            logger_printf("SMC dt_ == 0!\r\n");
            return 0.0f;
        }

        measure_ = measure;
        target_ = target;

        lambda_fast_ = lambda_ * q_ / (p_ == 0.0f ? 1.0f : p_);

        error_ = measure_ - target_;

        // 防止 error_ 恰好为 0 导致后续除零
        if (fabsf(error_) < 1e-6f)
        {
            error_ = copysignf(1e-6f, error_ == 0.0f ? 1.0f : error_); // 保持符号
        }

        omega_target_dot_ = (target_ - target_last_) / dt_;
        omega_target_dot_dot_ = (omega_target_dot_ - omega_target_dot_last_) / dt_;

        float raw_e_dot = (error_ - last_error_) / dt_;

        float ad = alpha_dot;
        if (!(ad >= 0.0f && ad <= 1.0f))
            ad = alpha_; // 兜底
        e_dot = (1.0f - ad) * e_dot_last + ad * raw_e_dot;

        // 防止 e_dot 接近 0 导致后续除零
        if (fabsf(e_dot) < 1e-9f)
            e_dot = copysignf(1e-9f, e_dot);

        // 有效 epsilon（避免 beta_ * fabs(error_) 导致除零）
        eps_eff_ = epsilon_ / (1.0f + beta_ * fabsf(error_));

        // 计算 e_qp：用安全的 pow
        float exponent = (p_ == 0.0f) ? q_ : (q_ / p_);
        float abs_err = fabsf(error_);
        e_qp_ = (error_ < 0.0f) ? -safe_powf(abs_err, exponent) : safe_powf(abs_err, exponent);

        // 主控制律
        if (fabsf(error_) <= dead_band_)
        {
            s_ = e_dot + lambda_ * error_;
            output_ = J_ * (omega_target_dot_dot_ - lambda_ * e_dot - k_ * s_ - eps_eff_ * sat(s_))+forward;
        }
        else
        {
            // 防止 fabs(error_) 为 0（已处理），并且确保除法项受控
            float denom = fabsf(error_);
            if (denom < 1e-9f)
                denom = 1e-9f;
            s_ = e_dot + lambda_ * e_qp_;
            output_ = J_ * (omega_target_dot_dot_ - lambda_fast_ * e_qp_ * e_dot / denom - k_ * s_ - eps_eff_ *
                sat(s_))+forward;
        }

        // 检查 output 是否为 NaN/Inf
        if (std::isnan(output_) || std::isinf(output_))
        {
//            logger_printf("SMC output is nan/inf! output=%f, err=%f, edot=%f\r\n", output_, error_, e_dot);

            output_ = 0.0f;
        }

        // 更新历史
        e_dot_last = e_dot;
        last_error_ = error_;
        last_output_ = output_;
        target_last_ = target_;
        omega_target_dot_last_ = omega_target_dot_;

        return output_;
    }

    void SMI_SISO::clear()
    {
        output_ = 0.0f;
        last_output_ = 0.0f;
        last_error_ = 0.0f;
        // 也清历史速度相关变量
        e_dot = 0.0f;
        e_dot_last = 0.0f;
        target_last_ = 0.0f;
        omega_target_dot_ = 0.0f;
        omega_target_dot_last_ = 0.0f;
        omega_target_dot_dot_ = 0.0f;
    }
}
