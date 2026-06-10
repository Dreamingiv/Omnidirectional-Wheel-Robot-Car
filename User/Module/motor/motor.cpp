#include "motor.h"

#include "DJIMotor.h"
#include "DMMotor.h"
#include "LKMotor.h"
#include "XYTmotor/XYTMotor.h"

namespace ega
{
    void Motor::controlAll()
    {
        // 维持原有结构：基类统一调度到各品牌的批处理
        DJIMotor::controlAll();
        DMMotor::controlAll();
        LKMotor::controlAll();
        XYTMotor::controlAll();
    }

    void Motor::enableAll()
    {
        DJIMotor::enableAll();
        DMMotor::enableAll();
        LKMotor::enableAll();
        XYTMotor::enableAll();
    }

    void Motor::disableAll()
    {
        DJIMotor::disableAll();
        DMMotor::disableAll();
        LKMotor::disableAll();
        XYTMotor::disableAll();
    }

    void Motor::syncEnableStateAll()
    {
        DJIMotor::syncEnableStateAll();
        DMMotor::syncEnableStateAll();
        LKMotor::syncEnableStateAll();
        XYTMotor::syncEnableStateAll();
    }


    bool Motor::hasDisabledMotor()
    {
        return DJIMotor::hasDisabledMotor()
            || DMMotor::hasDisabledMotor()
            || LKMotor::hasDisabledMotor()
            || XYTMotor::hasDisabledMotor();
    }

    bool Motor::hasOfflineMotor()
    {
        return DJIMotor::hasOfflineMotor()
            || DMMotor::hasOfflineMotor()
            || LKMotor::hasOfflineMotor()
            || XYTMotor::hasOfflineMotor();
    }
} // namespace ega
