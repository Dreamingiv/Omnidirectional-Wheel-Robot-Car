//
// Created by zhangzhiwen on 25-11-25.
//

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "matrix.h"

struct Empty
{
};

template <int x_hatSize, int zSize>
class KalmanFilter_without_U
{
public:
    KalmanFilter_without_U(Matrixf<x_hatSize, x_hatSize> Q, Matrixf<zSize, zSize> R) : Q_(Q), R_(R) {}

    KalmanFilter_without_U(float Q, float R)
    {
        Q_ = matrixf::eye<x_hatSize, x_hatSize>() * Q;
        R_ = matrixf::eye<zSize, zSize>() * R;
    }

    Matrixf<x_hatSize, 1> update(Matrixf<zSize, 1> measure)
    {
        z_ = measure;
        x_hat_minusUpdate();
        P_minusUpdate();
        setK();
        x_hatUpdate();
        P_Update();
        for (int i = 0; i < x_hatSize; i++)
        {
            if (P_(i, i) < 1e-6)
            {
                P_(i, i) = 1e-6;
            }
        }
        return x_hat_;
    }

private:
    void x_hat_minusUpdate() { x_hat_minus_ = F_ * x_hat_; }

    void P_minusUpdate() { P_minus_ = F_ * P_ * F_.trans() + Q_; }

    void setK() { K_ = P_minus_ * H_.trans() * matrixf::inv(H_ * P_minus_ * H_.trans() + R_); }

    void x_hatUpdate() { x_hat_ = x_hat_minus_ + K_ * (z_ - H_ * x_hat_minus_); }

    void P_Update() { P_ = (matrixf::eye<x_hatSize, x_hatSize>() - K_ * H_) * P_minus_; }

public:
    Matrixf<x_hatSize, 1>         x_hat_;
    Matrixf<x_hatSize, 1>         x_hat_minus_;
    Matrixf<zSize, 1>             z_;
    Matrixf<x_hatSize, x_hatSize> P_;
    Matrixf<x_hatSize, x_hatSize> P_minus_;
    Matrixf<x_hatSize, x_hatSize> F_;
    Matrixf<zSize, x_hatSize>     H_;
    Matrixf<x_hatSize, x_hatSize> Q_;
    Matrixf<zSize, zSize>         R_;
    Matrixf<x_hatSize, zSize>     K_;
};

template <int x_hatSize, int uSize, int zSize>
class KalmanFilter_with_U
{
public:
    KalmanFilter_with_U(Matrixf<x_hatSize, x_hatSize> Q, Matrixf<zSize, zSize> R) : Q_(Q), R_(R) {}

    KalmanFilter_with_U(float Q, float R)
    {
        Q_ = matrixf::eye<x_hatSize, x_hatSize>() * Q;
        R_ = matrixf::eye<zSize, zSize>() * R;
    }

    Matrixf<x_hatSize, 1> update(Matrixf<zSize, 1> measure)
    {
        z_ = measure;
        x_hat_minusUpdate();
        P_minusUpdate();
        setK();
        x_hatUpdate();
        P_Update();
        for (int i = 0; i < x_hatSize; i++)
        {
            if (P_(i, i) < 1e-6)
            {
                P_(i, i) = 1e-6;
            }
        }
        return x_hat_;
    }

    Matrixf<x_hatSize, 1> update(Matrixf<zSize, 1> measure, Matrixf<uSize, 1> u)
    {
        z_ = measure;
        u_ = u;
        x_hat_minusUpdate();
        P_minusUpdate();
        setK();
        x_hatUpdate();
        P_Update();
        for (int i = 0; i < x_hatSize; i++)
        {
            if (P_(i, i) < 1e-6)
            {
                P_(i, i) = 1e-6;
            }
        }
        return x_hat_;
    }

private:
    void x_hat_minusUpdate() { x_hat_minus_ = F_ * x_hat_ + B_ * u_; }

    void P_minusUpdate() { P_minus_ = F_ * P_ * F_.trans() + Q_; }

    void setK() { K_ = P_minus_ * H_.trans() * matrixf::inv(H_ * P_minus_ * H_.trans() + R_); }

    void x_hatUpdate() { x_hat_ = x_hat_minus_ + K_ * (z_ - H_ * x_hat_minus_); }

    void P_Update() { P_ = (matrixf::eye<x_hatSize, x_hatSize>() - K_ * H_) * P_minus_; }

public:
    Matrixf<x_hatSize, 1>         x_hat_;
    Matrixf<x_hatSize, 1>         x_hat_minus_;
    Matrixf<zSize, 1>             z_;
    Matrixf<x_hatSize, x_hatSize> P_;
    Matrixf<x_hatSize, x_hatSize> P_minus_;
    Matrixf<x_hatSize, x_hatSize> F_;
    Matrixf<zSize, x_hatSize>     H_;
    Matrixf<x_hatSize, x_hatSize> Q_;
    Matrixf<zSize, zSize>         R_;
    Matrixf<x_hatSize, zSize>     K_;

    Matrixf<uSize, 1>         u_;
    Matrixf<x_hatSize, uSize> B_;
};

#endif // KALMAN_FILTER_H
