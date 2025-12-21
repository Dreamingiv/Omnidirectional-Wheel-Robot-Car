//
// Created by zhangzhiwen on 25-11-25.
//

#ifndef QUATERNIONEKF_H
#define QUATERNIONEKF_H

#include "eulerAngle_and_quaternion.h"

/**
 * @brief 自定义1/sqrt(x),速度更快
 *
 * @param x x
 * @return float
 */
inline float invSqrt(float x)
{
    float half_x = 0.5f * x;
    float y = x;
    long i = *reinterpret_cast<long*>(&y);
    i = 0x5f375a86 - (i >> 1);
    y = *reinterpret_cast<float*>(&i);
    y = y * (1.5f - (half_x * y * y));
    return y;
}

class QuaternionEKF
{
public:
    QuaternionEKF(const QuaternionF32& init_quaternion,
                  const float process_noise_1,
                  const float process_noise_2,
                  const float measure_noise,
                  const float lambda,
                  const float lpf,
                  const float dt)
        : process_noise_1_(process_noise_1),
          process_noise_2_(process_noise_2),
          measure_noise_(measure_noise),
          lambda_(lambda > 1 ? 1 : lambda),
          lpf_(lpf),
          dt_(dt),
          q_(init_quaternion),
          euler_angle_(init_quaternion.toEuler())
    {
        x_hat_(0, 0) = init_quaternion.w;
        x_hat_(1, 0) = init_quaternion.x;
        x_hat_(2, 0) = init_quaternion.y;
        x_hat_(3, 0) = init_quaternion.z;

        F_ = MatrixF32::eye<6, 6>();
        P_ = Mat_f32<6, 6>(QuaternionEKF_P);
        R_ = MatrixF32::eye<3, 3>() * measure_noise_;
    }

    void update(Mat_f32<3, 1>& a, Mat_f32<3, 1>& g, const float dt)
    {
        static float half_gx_dt, half_gy_dt, half_gz_dt;
        static float accel_inv_norm;
        static EulerAngle e;

        dt_ = dt;

        g_ = g - g_bias_;
        // set F
        half_gx_dt = 0.5f * g_(0, 0) * dt_;
        half_gy_dt = 0.5f * g_(1, 0) * dt_;
        half_gz_dt = 0.5f * g_(2, 0) * dt_;

        F_ = MatrixF32::eye<6, 6>();

        F_(0, 1) = -half_gx_dt;
        F_(0, 2) = -half_gy_dt;
        F_(0, 3) = -half_gz_dt;

        F_(1, 0) = half_gx_dt;
        F_(1, 2) = half_gz_dt;
        F_(1, 3) = -half_gy_dt;

        F_(2, 0) = half_gy_dt;
        F_(2, 1) = -half_gz_dt;
        F_(2, 3) = half_gx_dt;

        F_(3, 0) = half_gz_dt;
        F_(3, 1) = -half_gy_dt;
        F_(3, 2) = -half_gx_dt;

        if (update_count_ == 0)
        {
            a_ = a;
        }
        a_ = a_ * (lpf_ / (dt_ + lpf_)) + a * (dt_ / (dt_ + lpf_));

        accel_inv_norm = invSqrt(
            a_(0, 0) * a_(0, 0)
            + a_(1, 0) * a_(1, 0)
            + a_(2, 0) * a_(2, 0)
        );
        // set z,单位化重力加速度向量
        z_ = a_ * accel_inv_norm;
        // get body state
        if ((g_.norm() < 0.3f) && (a_.norm() > (9.8f - 0.5f)) && (a_.norm() < (9.8f + 0.5f)))
        {
            stable_flag_ = true;
        }
        else
        {
            stable_flag_ = false;
        }
        // set Q R,过程噪声和观测噪声矩阵
        Q_ = MatrixF32::eye<6, 6>() * (process_noise_1_ * dt_);
        Q_(4, 4) = process_noise_2_ * dt_;
        Q_(5, 5) = process_noise_2_ * dt_;

        x_hat_minus_ = F_ * x_hat_;
        F_Linearization_P_Fading();
        P_minus_ = F_ * P_ * F_.trans() + Q_;
        setH();
        x_hat_Update();
        P_Update();

        q_.w = x_hat_(0, 0);
        q_.x = x_hat_(1, 0);
        q_.y = x_hat_(2, 0);
        q_.z = x_hat_(3, 0);
        q_.normalize();

        g_bias_(0, 0) = x_hat_(4, 0);
        g_bias_(1, 0) = x_hat_(5, 0);
        g_bias_(2, 0) = 0;

        e = q_.toEuler();
        euler_angle_.roll_ = e.roll_;
        euler_angle_.pitch_ = e.pitch_;
        euler_angle_.yaw_ = e.yaw_;
        euler_angle_.updateEulerAngle();

        update_count_++;
    }

    [[nodiscard]] QuaternionF32 getQuaternion() const { return q_; }
    [[nodiscard]] EulerAngle getEulerAngle() const { return euler_angle_; }

private:
    void F_Linearization_P_Fading()
    {
        static float q0, q1, q2, q3;
        static float q_inv_norm;

        q0 = x_hat_minus_(0, 0);
        q1 = x_hat_minus_(1, 0);
        q2 = x_hat_minus_(2, 0);
        q3 = x_hat_minus_(3, 0);
        q_inv_norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        x_hat_minus_(0, 0) *= q_inv_norm;
        x_hat_minus_(1, 0) *= q_inv_norm;
        x_hat_minus_(2, 0) *= q_inv_norm;
        x_hat_minus_(3, 0) *= q_inv_norm;
        // set F
        F_(0, 4) = q1 * dt_ / 2;
        F_(0, 5) = q2 * dt_ / 2;

        F_(1, 4) = -q0 * dt_ / 2;
        F_(1, 5) = q3 * dt_ / 2;

        F_(2, 4) = -q3 * dt_ / 2;
        F_(2, 5) = -q0 * dt_ / 2;

        F_(3, 4) = q2 * dt_ / 2;
        F_(3, 5) = -q1 * dt_ / 2;

        // fading filter,防止零飘参数过度收敛
        P_(4, 4) /= lambda_;
        P_(5, 5) /= lambda_;

        // 限幅,防止发散
        if (P_(4, 4) > 10000)
        {
            P_(4, 4) = 10000;
        }
        if (P_(5, 5) > 10000)
        {
            P_(5, 5) = 10000;
        }
    }

    void setH()
    {
        static float double_q0, double_q1, double_q2, double_q3;
        // set H
        double_q0 = 2 * x_hat_minus_(0, 0);
        double_q1 = 2 * x_hat_minus_(1, 0);
        double_q2 = 2 * x_hat_minus_(2, 0);
        double_q3 = 2 * x_hat_minus_(3, 0);
        float H_data[18] = {
            -double_q2, double_q3, -double_q0, double_q1, 0, 0,
            double_q1, double_q0, double_q3, double_q2, 0, 0,
            double_q0, -double_q1, -double_q2, double_q3, 0, 0
        };
        H_ = Mat_f32<3, 6>(H_data);
    }

    void x_hat_Update()
    {
        static float q0, q1, q2, q3;
        static Mat_f32<3, 3> temp_mat;
        static Mat_f32<3, 1> temp_vec;
        static Mat_f32<6, 1> temp_vec_1;
        static float orientation_cos[3]{};
        static Mat_f32<1, 1> chi_square;
        static float adaptive_gain_scale;

        q0 = x_hat_minus_(0, 0);
        q1 = x_hat_minus_(1, 0);
        q2 = x_hat_minus_(2, 0);
        q3 = x_hat_minus_(3, 0);

        temp_vec(0, 0) = 2 * (q1 * q3 - q0 * q2);
        temp_vec(1, 0) = 2 * (q0 * q1 + q2 * q3);
        temp_vec(2, 0) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        orientation_cos[0] = acosf(fabsf(temp_vec(0, 0)));
        orientation_cos[1] = acosf(fabsf(temp_vec(1, 0)));
        orientation_cos[2] = acosf(fabsf(temp_vec(2, 0)));

        temp_vec = z_ - temp_vec;
        temp_mat = MatrixF32::inv(H_ * P_minus_ * H_.trans() + R_);
        chi_square = temp_vec.trans() * temp_mat * temp_vec;
        // logger_printf("%f\r\n", chi_square(0, 0));
        // rk is small,filter converged/converging
        if (chi_square(0, 0) < 0.5f * chi_square_test_threshold_)
        {
            converge_flag_ = true;
        }
        // rk is bigger than threshold but once converged
        if (chi_square(0, 0) > chi_square_test_threshold_ && converge_flag_)
        {
            if (stable_flag_)
            {
                error_count_++;
            }
            else
            {
                error_count_ = 0;
            }

            if (error_count_ > 50)
            {
                converge_flag_ = false;
                P_update_flag_ = true;
            }
            else
            {
                x_hat_ = x_hat_minus_;
                P_ = P_minus_;
                P_update_flag_ = false;
            }
        }
        else // if divergent or rk is not that big/acceptable,use adaptive gain
        {
            // scale adaptive,rk越小则增益越大,否则更相信预测值
            if (chi_square(0, 0) > 0.1f * chi_square_test_threshold_ && converge_flag_)
            {
                adaptive_gain_scale = (chi_square_test_threshold_ - chi_square(0, 0)) / (0.9f *
                    chi_square_test_threshold_);
            }
            else
            {
                adaptive_gain_scale = 1;
            }
            error_count_ = 0;
            P_update_flag_ = true;
        }

        // cal kf-gain K
        K_ = P_minus_ * H_.trans() * temp_mat;
        // implement adaptive
        K_ *= adaptive_gain_scale;

        K_(4, 0) *= orientation_cos[0] / 1.5707963f;
        K_(4, 1) *= orientation_cos[0] / 1.5707963f;
        K_(4, 2) *= orientation_cos[0] / 1.5707963f;
        K_(5, 0) *= orientation_cos[1] / 1.5707963f;
        K_(5, 1) *= orientation_cos[1] / 1.5707963f;
        K_(5, 2) *= orientation_cos[1] / 1.5707963f;

        temp_vec_1 = K_ * temp_vec;
        if (converge_flag_)
        {
            if (temp_vec_1(4, 0) > 1e-2f * dt_) { temp_vec_1(4, 0) = 1e-2f * dt_; }
            if (temp_vec_1(4, 0) < -1e-2f * dt_) { temp_vec_1(4, 0) = -1e-2f * dt_; }
            if (temp_vec_1(5, 0) > 1e-2f * dt_) { temp_vec_1(5, 0) = 1e-2f * dt_; }
            if (temp_vec_1(5, 0) < -1e-2f * dt_) { temp_vec_1(5, 0) = -1e-2f * dt_; }
        }
        // 不修正yaw轴数据
        temp_vec_1(3, 0) = 0;

        x_hat_ = x_hat_minus_ + temp_vec_1;
    }

    void P_Update()
    {
        if (P_update_flag_)
        {
            P_ = P_minus_ - K_ * H_ * P_minus_;
        }
    }

    float process_noise_1_;
    float process_noise_2_;
    float measure_noise_;
    float lambda_;
    float lpf_;
    float dt_;

    const float chi_square_test_threshold_{1e-8f};
    bool stable_flag_{true};
    bool converge_flag_{false};
    bool P_update_flag_{false};
    size_t error_count_{0};
    size_t update_count_{0};

    Mat_f32<3, 1> a_{};
    Mat_f32<3, 1> g_{};
    Mat_f32<3, 1> g_bias_{};

    QuaternionF32 q_;
    EulerAngle euler_angle_;
    Mat_f32<6, 1> x_hat_;
    Mat_f32<6, 1> x_hat_minus_;
    Mat_f32<3, 1> z_;
    Mat_f32<6, 6> P_;
    Mat_f32<6, 6> P_minus_;
    Mat_f32<6, 6> F_;
    Mat_f32<3, 6> H_;
    Mat_f32<6, 6> Q_;
    Mat_f32<3, 3> R_;
    Mat_f32<6, 3> K_;

    constexpr static float QuaternionEKF_P[36] =
    {
        100000, 0.1, 0.1, 0.1, 0.1, 0.1,
        0.1, 100000, 0.1, 0.1, 0.1, 0.1,
        0.1, 0.1, 100000, 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1, 100000, 0.1, 0.1,
        0.1, 0.1, 0.1, 0.1, 100, 0.1,
        0.1, 0.1, 0.1, 0.1, 0.1, 100
    };
};


#endif //QUATERNIONEKF_H
