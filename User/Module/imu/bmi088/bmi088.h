//
// Created by zhangzhiwen on 25-10-13.
//

#ifndef MODULE_IMU_BMI088_H_
#define MODULE_IMU_BMI088_H_

#include <optional>
#include "imu.h"
#include "driver_spi.h"
#include "madgwick_filter.h"
#include "bmi088reg.h"

namespace ega
{
    class BMI088 : public IMU
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        enum ErrorType
        {
            BMI088_NO_ERROR = 0x00,
            BMI088_ACC_PWR_CTRL_ERROR = 0x01,
            BMI088_ACC_PWR_CONF_ERROR = 0x02,
            BMI088_ACC_CONF_ERROR = 0x03,
            BMI088_ACC_SELF_TEST_ERROR = 0x04,
            BMI088_ACC_RANGE_ERROR = 0x05,
            BMI088_INT1_IO_CTRL_ERROR = 0x06,
            BMI088_INT_MAP_DATA_ERROR = 0x07,
            BMI088_GYRO_RANGE_ERROR = 0x08,
            BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
            BMI088_GYRO_LPM1_ERROR = 0x0A,
            BMI088_GYRO_CTRL_ERROR = 0x0B,
            BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
            BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,
            BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
            BMI088_SELF_TEST_GYRO_ERROR = 0x40,
            BMI088_NO_SENSOR = 0xFF,
        };

        /* ====================== 3. 静态接口 ====================== */

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        explicit BMI088(const IMU::Config& config);
        ~BMI088() override = default;
        /* ====================== 5. 公共接口 ====================== */
    public:
        // IMU 基类接口实现
        ErrorType init();
        void readData() override;
        void calibrate() override;
        void calculate() override;

        // 辅助函数
        [[nodiscard]] ErrorType getError() const { return error_code_; }
        [[nodiscard]] float getTemperature() const { return temperature_; };
        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        std::optional<SPIInstance> spi_accel_;
        std::optional<SPIInstance> spi_gyro_;
        // std::optional<MadgwickFilter> filter_;
        ErrorType error_code_;

        float accel_sensitivity_;
        float gyro_sensitivity_;
        float accel_scale_;
        float gyro_offset_[3];
        float temperature_;

        const uint8_t accel_init_table[BMI088_WRITE_ACCEL_REG_NUM][3] =
        {
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
            {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
            {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
            {
                BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW,
                BMI088_INT1_IO_CTRL_ERROR
            },
            {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
        };

        const uint8_t gyro_init_table[BMI088_WRITE_GYRO_REG_NUM][3] =
        {
            {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
            {
                BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
                BMI088_GYRO_BANDWIDTH_ERROR
            },
            {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
            {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
            {
                BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
                BMI088_GYRO_INT3_INT4_IO_CONF_ERROR
            },
            {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
        };
    private:
        // 内部功能函数
        ErrorType accelInit();
        ErrorType gyroInit();
        void readAccel();
        void readGyro();
        void readTemperature();

        // === 底层读写函数===
        inline void accelWriteReg(uint8_t reg, uint8_t data);
        inline uint8_t accelReadReg(uint8_t reg);
        inline void accelReadRegs(uint8_t reg, uint8_t* data, uint16_t len);

        inline void gyroWriteReg(uint8_t reg, uint8_t data);
        inline uint8_t gyroReadReg(uint8_t reg);
        inline void gyroReadRegs(uint8_t reg, uint8_t* data, uint16_t len);
    };
}


#endif //MODULE_IMU_BMI088_H_
