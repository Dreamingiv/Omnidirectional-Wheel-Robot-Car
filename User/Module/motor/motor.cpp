#include "motor.h"

#include "DJImotor.h"
#include "DMmotor.h"

namespace ega
{
    void Motor::controlAll()
    {
        DJIMotor::controlAll();
        DMMotor::controlAll();
    }

    void Motor::enableAll()
    {
        DJIMotor::enableAll();
        DMMotor::enableAll();
    }

    void Motor::disableAll()
    {
        DJIMotor::disableAll();
        DMMotor::disableAll();
    }

    bool Motor::hasDisabledMotor()
    {
        //todo
        return false;
    }

    bool Motor::hasOfflineMotor()
    {
        //todo
        return false;
    }


    std::unique_ptr<Motor> Motor::create(const Motor::Config& config)
    {
        switch (config.type)
        {
        case Type::GM6020:
        case Type::M3508:
        case Type::M2006:
            return std::make_unique<DJIMotor>(config);

        case Type::DM_MOTOR:
            return std::make_unique<DMMotor>(config);

        case Type::OTHER:
        default:
            return nullptr;
        }
    }
}
