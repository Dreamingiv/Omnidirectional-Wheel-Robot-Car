#include "motor.h"

#include "DJIMotor.h"
#include "DMMotor.h"
#include "LKMotor.h"

namespace ega
{
    void Motor::controlAll()
    {
        // 维持原有结构：基类统一调度到各品牌的批处理
        DJIMotor::controlAll();
        DMMotor::controlAll();
        LKMotor::controlAll();
    }

    void Motor::enableAll()
    {
        DJIMotor::enableAll();
        DMMotor::enableAll();
        LKMotor::enableAll();
    }

    void Motor::disableAll()
    {
        DJIMotor::disableAll();
        DMMotor::disableAll();
        LKMotor::disableAll();
    }

    bool Motor::hasDisabledMotor()
    {
        return DJIMotor::hasDisabledMotor()
            || DMMotor::hasDisabledMotor()
            || LKMotor::hasDisabledMotor();
    }

    bool Motor::hasOfflineMotor()
    {
        return DJIMotor::hasOfflineMotor()
            || DMMotor::hasOfflineMotor()
            || LKMotor::hasOfflineMotor();
    }
} // namespace ega
