//
// Created by zhangzhiwen on 25-11-25.
//

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "matrix.h"

struct Empty
{
};

template <int x_hatSize, int uSize, int zSize>
class KalmanFilter
{
    using Mat_u = std::conditional_t<(uSize > 0), Mat_f32<uSize, 1>, Empty>;
    using Mat_B = std::conditional_t<(uSize > 0), Mat_f32<x_hatSize, uSize>, Empty>;

public:
    KalmanFilter(Mat_f32<x_hatSize, x_hatSize> Q,
                 Mat_f32<zSize, zSize>         R)
        : Q_(Q), R_(R)
    {
    }

    KalmanFilter(float Q, float R)
    {
        Q_ = MatrixF32::eye<x_hatSize, x_hatSize>() * Q;
        R_ = MatrixF32::eye<zSize, zSize>() * R;
    }

    Mat_f32<x_hatSize, 1> update(Mat_f32<zSize, 1> measure)
    {
        z_ = measure;
        x_hat_minusUpdate();
        P_minusUpdate();
        setK();
        x_hatUpdate();
        P_Update();
        for (int i = 0; i < x_hatSize; i++)
        {
            if (P_(i, i) < 1e-6) { P_(i, i) = 1e-6; }
        }
        return x_hat_;
    }

    Mat_f32<x_hatSize, 1> update(Mat_f32<zSize, 1> measure, Mat_u u)
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
            if (P_(i, i) < 1e-6) { P_(i, i) = 1e-6; }
        }
        return x_hat_;
    }

private:
    void x_hat_minusUpdate()
    {
        if constexpr (uSize > 0)
        {
            x_hat_minus_ = F_ * x_hat_ + B_ * u_;
        }
        else
        {
            x_hat_minus_ = F_ * x_hat_;
        }
    }

    void P_minusUpdate()
    {
        P_minus_ = F_ * P_ * F_.trans() + Q_;
    }

    void setK()
    {
        K_ = P_minus_ * H_.trans() * MatrixF32::inv(H_ * P_minus_ * H_.trans() + R_);
    }

    void x_hatUpdate()
    {
        x_hat_ = x_hat_minus_ + K_ * (z_ - H_ * x_hat_minus_);
    }

    void P_Update()
    {
        P_ = (MatrixF32::eye<x_hatSize, x_hatSize>() - K_ * H_) * P_minus_;
    }

    Mat_f32<x_hatSize, 1>         x_hat_;
    Mat_f32<x_hatSize, 1>         x_hat_minus_;
    Mat_f32<zSize, 1>             z_;
    Mat_f32<x_hatSize, x_hatSize> P_;
    Mat_f32<x_hatSize, x_hatSize> P_minus_;
    Mat_f32<x_hatSize, x_hatSize> F_;
    Mat_f32<zSize, x_hatSize>     H_;
    Mat_f32<x_hatSize, x_hatSize> Q_;
    Mat_f32<zSize, zSize>         R_;
    Mat_f32<x_hatSize, zSize>     K_;
    // Mat_f32<x_hatSize, x_hatSize> S_;
    // Mat_f32<x_hatSize, x_hatSize> temp_mat_0_;
    // Mat_f32<x_hatSize, x_hatSize> temp_mat_1_;
    // Mat_f32<x_hatSize, 1>         tmep_vec_0_;
    // Mat_f32<x_hatSize, 1>         tmep_vec_1_;

    Mat_u u_;
    Mat_B B_;
};

#endif //KALMAN_FILTER_H
