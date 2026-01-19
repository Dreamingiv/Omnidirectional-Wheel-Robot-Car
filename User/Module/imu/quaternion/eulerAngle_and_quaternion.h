//
// Created by zhangzhiwen on 25-10-12.
//

#ifndef EULERANGLE_AND_QUATERNION_H
#define EULERANGLE_AND_QUATERNION_H

#include "matrix.h"

class EulerAngle
{
public:
    EulerAngle() = default;

    EulerAngle(const float init_roll, const float init_pitch, const float init_yaw)
        : roll_(init_roll), pitch_(init_pitch), yaw_(init_yaw),
          roll_last_(init_roll), pitch_last_(init_pitch), yaw_last_(init_yaw),
          total_roll_(init_roll), total_pitch_(init_pitch), total_yaw_(init_yaw),
          round_roll_(0), round_pitch_(0), round_yaw_(0)
    {
    }

    // 累计总角度
    void updateEulerAngle()
    {
        updateRoll();
        updatePitch();
        updateYaw();
    }

    float roll_{0},       pitch_{0},       yaw_{0};       // rad
    float roll_last_{0},  pitch_last_{0},  yaw_last_{0};  // rad
    float total_roll_{0}, total_pitch_{0}, total_yaw_{0}; // rad
    int   round_roll_{0}, round_pitch_{0}, round_yaw_{0}; // spin time
private:
    static inline float update(const float angle, float& angle_last, int& round)
    {
        if (angle - angle_last < -6)
        {
            round++;
        }
        else if (angle - angle_last > 6)
        {
            round--;
        }
        angle_last = angle;
        return angle + static_cast<float>(round) * PI * 2.0f;
    }

    void updateRoll()
    {
        this->total_roll_ = update(this->roll_, this->roll_last_, this->round_roll_);
    }

    void updatePitch()
    {
        this->total_pitch_ = update(this->pitch_, this->pitch_last_, this->round_pitch_);
    }

    void updateYaw()
    {
        this->total_yaw_ = update(this->yaw_, this->yaw_last_, this->round_yaw_);
    }
};

class QuaternionF32
{
public:
    float w, x, y, z;

    // ----------- 构造函数 -----------
    explicit QuaternionF32(float w_ = 1.0f, float x_ = 0.0f, float y_ = 0.0f, float z_ = 0.0f)
        : w(w_), x(x_), y(y_), z(z_)
    {
    }

    QuaternionF32(Matrixf<3, 1>& axis, float angle_rad)
    {
        float half = angle_rad * 0.5f;
        float s    = arm_sin_f32(half);

        w = arm_cos_f32(half);
        x = axis(0, 0) * s;
        y = axis(1, 0) * s;
        z = axis(2, 0) * s;
        normalize();
    }

    explicit QuaternionF32(Matrixf<4, 1>& m)
        : w(m(0, 0)), x(m(1, 1)), y(m(2, 2)), z(m(3, 3))
    {
    }

    // ----------- 基本操作 -----------
    void normalize()
    {
        float norm = sqrtf(w * w + x * x + y * y + z * z);
        if (norm > 1e-8f)
        {
            float inv = 1.0f / norm;

            w *= inv;
            x *= inv;
            y *= inv;
            z *= inv;
        }
    }

    [[nodiscard]] QuaternionF32 conjugate() const { return QuaternionF32(w, -x, -y, -z); }

    [[nodiscard]] QuaternionF32 inverse() const
    {
        float norm2 = w * w + x * x + y * y + z * z;
        if (norm2 < 1e-8f)
            return QuaternionF32();
        float inv = 1.0f / norm2;
        return QuaternionF32(w * inv, -x * inv, -y * inv, -z * inv);
    }

    // ----------- 四元数运算 -----------
    QuaternionF32 operator*(const QuaternionF32& q) const
    {
        return QuaternionF32(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    QuaternionF32& operator*=(const QuaternionF32& q)
    {
        *this = (*this) * q;
        return *this;
    }

    QuaternionF32& operator+=(const QuaternionF32& q)
    {
        *this = (*this) + q;
        return *this;
    }

    QuaternionF32 operator*(float s) const { return QuaternionF32(w * s, x * s, y * s, z * s); }
    QuaternionF32 operator+(const QuaternionF32& q) const { return QuaternionF32(w + q.w, x + q.x, y + q.y, z + q.z); }
    QuaternionF32 operator-(const QuaternionF32& q) const { return QuaternionF32(w - q.w, x - q.x, y - q.y, z - q.z); }

    // ----------- 与向量操作 -----------
    Matrixf<3, 1> rotate(Matrixf<3, 1>& v) const
    {
        QuaternionF32 qv(0, v(0, 0), v(1, 0), v(2, 0));
        QuaternionF32 qr = (*this) * qv * this->conjugate();
        Matrixf<3, 1> res;
        res(0, 0) = qr.x;
        res(1, 0) = qr.y;
        res(2, 0) = qr.z;
        return res;
    }

    // ----------- 转换函数 -----------
    [[nodiscard]] Matrixf<3, 3> toRotationMatrix() const
    {
        Matrixf<3, 3> R;

        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;

        R(0, 0) = 1 - 2 * (yy + zz);
        R(0, 1) = 2 * (xy - wz);
        R(0, 2) = 2 * (xz + wy);

        R(1, 0) = 2 * (xy + wz);
        R(1, 1) = 1 - 2 * (xx + zz);
        R(1, 2) = 2 * (yz - wx);

        R(2, 0) = 2 * (xz - wy);
        R(2, 1) = 2 * (yz + wx);
        R(2, 2) = 1 - 2 * (xx + yy);
        return R;
    }

    static QuaternionF32 fromRotationMatrix(Matrixf<3, 3>& R)
    {
        QuaternionF32 q;
        float         trace = R(0, 0) + R(1, 1) + R(2, 2);
        if (trace > 0.0f)
        {
            float s = sqrtf(trace + 1.0f) * 2.0f;
            q.w     = 0.25f * s;
            q.x     = (R(2, 1) - R(1, 2)) / s;
            q.y     = (R(0, 2) - R(2, 0)) / s;
            q.z     = (R(1, 0) - R(0, 1)) / s;
        }
        else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2))
        {
            float s = sqrtf(1.0f + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0f;
            q.w     = (R(2, 1) - R(1, 2)) / s;
            q.x     = 0.25f * s;
            q.y     = (R(0, 1) + R(1, 0)) / s;
            q.z     = (R(0, 2) + R(2, 0)) / s;
        }
        else if (R(1, 1) > R(2, 2))
        {
            float s = sqrtf(1.0f + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0f;
            q.w     = (R(0, 2) - R(2, 0)) / s;
            q.x     = (R(0, 1) + R(1, 0)) / s;
            q.y     = 0.25f * s;
            q.z     = (R(1, 2) + R(2, 1)) / s;
        }
        else
        {
            float s = sqrtf(1.0f + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0f;
            q.w     = (R(1, 0) - R(0, 1)) / s;
            q.x     = (R(0, 2) + R(2, 0)) / s;
            q.y     = (R(1, 2) + R(2, 1)) / s;
            q.z     = 0.25f * s;
        }
        q.normalize();
        return q;
    }

    // ----------- 欧拉角互转 -----------
    static QuaternionF32 fromEuler(float roll, float pitch, float yaw)
    {
        float cr = arm_cos_f32(roll * 0.5f);
        float sr = arm_sin_f32(roll * 0.5f);
        float cp = arm_cos_f32(pitch * 0.5f);
        float sp = arm_sin_f32(pitch * 0.5f);
        float cy = arm_cos_f32(yaw * 0.5f);
        float sy = arm_sin_f32(yaw * 0.5f);

        QuaternionF32 q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }

    static QuaternionF32 fromEuler(EulerAngle euler)
    {
        float roll  = euler.roll_;
        float pitch = euler.pitch_;
        float yaw   = euler.yaw_;

        float cr = arm_cos_f32(roll * 0.5f);
        float sr = arm_sin_f32(roll * 0.5f);
        float cp = arm_cos_f32(pitch * 0.5f);
        float sp = arm_sin_f32(pitch * 0.5f);
        float cy = arm_cos_f32(yaw * 0.5f);
        float sy = arm_sin_f32(yaw * 0.5f);

        QuaternionF32 q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }

    [[nodiscard]] EulerAngle toEuler() const
    {
        EulerAngle euler;
        float      sinr_cosp = 2 * (w * x + y * z);
        float      cosr_cosp = 1 - 2 * (x * x + y * y);

        euler.roll_ = atan2f(sinr_cosp, cosr_cosp);

        float sinp = 2 * (w * y - z * x);

        euler.pitch_ = (fabsf(sinp) >= 1) ? copysignf(M_PI / 2, sinp) : asinf(sinp);

        float siny_cosp = 2 * (w * z + x * y);
        float cosy_cosp = 1 - 2 * (y * y + z * z);

        euler.yaw_ = atan2f(siny_cosp, cosy_cosp);
        euler.updateEulerAngle();
        return euler;
    }

    [[nodiscard]] Matrixf<3, 1> toAccel() const
    {
        Matrixf<3, 1> accel;

        accel(0, 0) = 2.0f * (x * z - w * y);
        accel(1, 0) = 2.0f * (w * x + y * z);
        accel(2, 0) = w * w - x * x - y * y + z * z;
        return accel;
    }

    [[nodiscard]] Matrixf<3, 4> toJacobian() const
    {
        Matrixf<3, 4> jacobian;
        jacobian(0, 0) = -2 * y;
        jacobian(0, 1) = 2 * z;
        jacobian(0, 2) = -2 * w;
        jacobian(0, 3) = 2 * x;
        jacobian(1, 0) = 2 * x;
        jacobian(1, 1) = 2 * w;
        jacobian(1, 2) = 2 * z;
        jacobian(1, 3) = 2 * y;
        jacobian(2, 0) = 0.0f;
        jacobian(2, 1) = -4 * x;
        jacobian(2, 2) = -4 * y;
        jacobian(2, 3) = 0.0f;
        return jacobian;
    }

    [[nodiscard]] QuaternionF32 toQdot(Matrixf<3, 1>& omega) const
    {
        QuaternionF32 omega_quaternion(
            0,
            0.5f * omega(0, 0),
            0.5f * omega(1, 0),
            0.5f * omega(2, 0));
        return (*this) * omega_quaternion;
    }

    // ----------- IMU积分更新（角速度）-----------
    void integrate(Matrixf<3, 1>& omega, float dt)
    {
        QuaternionF32 dq(0,
                         0.5f * omega(0, 0) * dt,
                         0.5f * omega(1, 0) * dt,
                         0.5f * omega(2, 0) * dt);
        *this = *this + dq * (*this);
        normalize();
    }
};

#endif //QUATERNION_F32_H
