//
// Created by An on 2025/11/21.
//
//
#ifndef DRIVER_UTILS_H_
#define DRIVER_UTILS_H_

#include "main.h"

namespace ega::utils
{
    /**
     * 绝对值、最大值、最小值、交换值等
     * clamp、map等数值映射
     * 大小端转换
     * 类型转换，float转int，int转float等
     *
     * 优先实现其他模块已经用到的
     */
    union data32
    {
        int32_t L;
        uint8_t u8[4];
        uint16_t u16[2];
        uint32_t u32;
        float F;
    };

    union data16
    {
        int16_t s;
        uint8_t u8[2];
        uint16_t u16;
    };

    //  绝对值
    inline float abs(const float x)
    {
        return x > 0 ? x : -x;
    }

    // 限幅
    inline float limit(const float x, const float min, const float max)
    {
        return x < min ? min : (x > max ? max : x);
    }

    // 符号函数
    inline float sign(const float x)
    {
        return (x > 0) ? 1.0f : ((x < 0) ? -1.0f : 0);
    }

    // 两位uint8_t拼接成uint16_t
    inline uint16_t u8_to_u16(const uint8_t u8_h, const uint8_t u8_l)
    {
        return static_cast<uint16_t>((u8_h << 8) | u8_l);
    }

    // uint16_t拆分成uint8_t
    inline void u16_to_u8(const uint16_t u16, uint8_t& u8_h, uint8_t& u8_l)
    {
        u8_h = u16 >> 8;
        u8_l = u16 & 0xFF;
    }

    // uint16_t按比例缩放成float
    inline float u16_to_float(uint16_t x_int, float x_min, float x_max, int bits)
    {
        float span = x_max - x_min;
        float offset = x_min;
        return static_cast<float>(x_int) * span /
            static_cast<float>((1 << bits) - 1) + offset;
    }

    // float按比例缩放成uint16_t
    inline uint16_t float_to_u16(float x, float x_min, float x_max, int bits)
    {
        float span = x_max - x_min;
        float offset = x_min;
        return static_cast<uint16_t>(((x - offset) * static_cast<float>((1 << bits) - 1)) / span);
    }

    inline uint32_t crc32_core_Ver3(const uint32_t* ptr, uint32_t len)
    {
        uint32_t bits;
        uint32_t i;
        uint32_t xbit = 0;
        uint32_t data = 0;
        uint32_t CRC32 = 0xFFFFFFFF;
        const uint32_t dwPolynomial = 0x04c11db7;
        for (i = 0; i < len; i++)
        {
            xbit = 1 << 31;
            data = ptr[i];
            for (bits = 0; bits < 32; bits++)
            {
                if (CRC32 & 0x80000000)
                {
                    CRC32 <<= 1;
                    CRC32 ^= dwPolynomial;
                }
                else
                    CRC32 <<= 1;
                if (data & xbit)
                    CRC32 ^= dwPolynomial;

                xbit >>= 1;
            }
        }
        return CRC32;
    }
}

#endif //DRIVER_UTILS_H_
