#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include "main.h"

#include "eulerAngle_and_quaternion.h"
#include "kalman_filter.h"
#include "matrix.h"

class MadgwickFilter
{
public:
    explicit MadgwickFilter(float beta = 0.002, float Q = 1e-3, float R = 1e+6) : beta_(beta), kf_(Q, R)
    {
        kf_.F_ = matrixf::eye<2, 2>();
        kf_.H_ = matrixf::eye<2, 2>();
    }

    void update(Matrixf<3, 1>& a, Matrixf<3, 1>& g, const float dt)
    {
        static QuaternionF32 dq;
        static Matrixf<3, 1> a_hat;
        static Matrixf<2, 1> g_hat;
        static Matrixf<2, 1> g_bias;
        static Matrixf<3, 4> J;
        static Matrixf<4, 1> s;

        a_ = a;
        g_ = g;
        kf_.F_ = matrixf::eye<2, 2>() * dt;

        dq = q_.toQdot(g_);

        g_hat(0, 0) = g_(0, 0) - 2 * dq.x;
        g_hat(1, 0) = g_(1, 0) - 2 * dq.y;

        g_bias = kf_.update(g_hat);
        g_bias_(0, 0) = g_bias(0, 0);
        g_bias_(1, 0) = g_bias(1, 0);
        g_bias_(2, 0) = 0;

        g_ -= g_bias_;

        dq = q_.toQdot(g_);

        float norm = 1.0f / a_.invNorm();
        if (9.4f < norm && norm < 10.2f)
        {
            a_.normalize();

            a_hat = q_.toAccel();
            a_hat.normalize();

            J = q_.toJacobian();

            s = J.trans() * (a_hat - a_);
            s.normalize();
            s *= beta_;

            dq = dq - QuaternionF32(s);
        }

        q_ = q_ + dq * dt;

        q_.normalize();

        e_raw = q_.toEuler();
        euler_angle_.roll_ = e_raw.roll_;
        euler_angle_.pitch_ = e_raw.pitch_;
        euler_angle_.yaw_ = e_raw.yaw_;
        euler_angle_.updateEulerAngle();
    }

    [[nodiscard]] QuaternionF32 getQuaternion() const { return q_; }
    [[nodiscard]] EulerAngle getEulerAngle() const { return euler_angle_; }
    [[nodiscard]] Matrixf<3, 1> getAccel() const { return a_; }
    [[nodiscard]] Matrixf<3, 1> getGyro() const { return g_; }

private:
    float beta_;

    KalmanFilter_without_U<2, 2> kf_;

    Matrixf<3, 1> a_{};
    Matrixf<3, 1> g_{};
    Matrixf<3, 1> g_bias_{};

    QuaternionF32 q_;
    EulerAngle e_raw;
    EulerAngle euler_angle_;

    static float InvSqrtf(float x)
    {
        /* Fast inverse square-root */
        /* See: http://en.wikipedia.org/wiki/Fast_inverse_square_root */
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i >> 1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        y = y * (1.5f - (halfx * y * y));
        return y;
    }
};


#endif
