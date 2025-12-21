//
// Created by zhangzhiwen on 25-10-14.
//

#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include "matrix.h"
#include "eulerAngle_and_quaternion.h"

class MadgwickFilter
{
public:
    MadgwickFilter(float beta = 0.1f, float samplePeriod = 0.001f);

    // 主更新函数（只使用陀螺仪 + 加速度计）
    void updateIMU(Mat_f32<3, 1> accel, Mat_f32<3, 1> gyro, QuaternionF32& q);

private:
    float beta_;         // 滤波增益系数
    float samplePeriod_; // 采样周期
};

#endif //MADGWICK_FILTER_H
