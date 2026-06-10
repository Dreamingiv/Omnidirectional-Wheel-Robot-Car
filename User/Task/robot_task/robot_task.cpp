//
// Created by An on 2025/10/24.
//
#include "robot_task.h"
#include "cmsis_os.h"
#include "chassis.h"
#include "imu.h"
#include "logger.h"
#include "minipc.h"
#include "PID.h"
#include "robot_manager.h"

#define CHASSIS_TEST_MODE

namespace
{
    constexpr float CHASSIS_TEST_SPEED = 0.90f;
    constexpr TickType_t CHASSIS_TEST_MOVE_TICKS = pdMS_TO_TICKS(2000);
    constexpr TickType_t CHASSIS_TEST_STOP_TICKS = pdMS_TO_TICKS(500);
    constexpr float YAW_OUTPUT_MAX = 3.0f;
    constexpr float YAW_COMMAND_DEADBAND = 0.02f;
    constexpr float YAW_GYRO_DAMPING = 0.18f;
    constexpr float LATERAL_ACCEL_DT = 0.005f;
    constexpr float LATERAL_ACCEL_DEADBAND = 0.08f;
    constexpr float LATERAL_VELOCITY_LEAK = 0.98f;
    constexpr float LATERAL_COMP_GAIN = 0.35f;
    constexpr float LATERAL_COMP_MAX = 0.18f;
    constexpr float LATERAL_BIAS_ALPHA = 0.01f;
    constexpr float LATERAL_STILL_COMMAND_DEADBAND = 0.04f;
    constexpr float LATERAL_STILL_GYRO_DEADBAND = 0.12f;
    constexpr float LATERAL_MIN_FORWARD_COMMAND = 0.15f;
    constexpr float LATERAL_MAX_SIDE_COMMAND = 0.10f;
    constexpr float LATERAL_MAX_WZ_COMMAND = 0.20f;
    constexpr float COMMAND_RAMP_DT = 0.005f;
    constexpr float COMMAND_RAMP_V_RATE = 4.0f;
    constexpr float COMMAND_RAMP_WZ_RATE = 4.0f;

    float limitFloat(float value, float min, float max)
    {
        if (value < min)
        {
            return min;
        }
        if (value > max)
        {
            return max;
        }
        return value;
    }

    float absFloat(float value)
    {
        return value >= 0.0f ? value : -value;
    }

    float approachFloat(float current, float target, float max_delta)
    {
        const float delta = target - current;
        if (delta > max_delta)
        {
            return current + max_delta;
        }
        if (delta < -max_delta)
        {
            return current - max_delta;
        }
        return target;
    }

    const ega::DT7* getDT7Remote()
    {
        if (!ega::RobotManager::hasRemote())
        {
            return nullptr;
        }

        const auto& remote = ega::RobotManager::getRemoteConst();
        return std::get_if<ega::DT7>(&remote);
    }

    ega::Chassis::Command toChassisCommand(const ega::MiniPC::Command& command)
    {
        return {
            .vx = command.vx,
            .vy = command.vy,
            .wz = command.wz,
        };
    }

    ega::Chassis::Command applyYawTargetControl(
        const ega::Chassis::Command& command,
        bool active,
        bool reset_yaw_target = false)
    {
        static ega::PID yaw_pid({
            .kp = 0.3f,
            .ki = 0.0f,
            .kd = 0.0f,
            .limit_output = YAW_OUTPUT_MAX,
            .limit_integral = 0.0f,
            .limit_error = 1.0f,
            .limit_diff = 0.0f,
            .dead_band = 0.02f,
            .dt = 5.0f,
        });
        static bool target_yaw_valid = false;
        static float target_yaw = 0.0f;

        ega::Chassis::Command output = command;

        float current_yaw = 0.0f;
        float gyro_z = 0.0f;
        if (!active ||
            !ega::IMU::getPrimaryYaw(current_yaw) ||
            !ega::IMU::getPrimaryGyroZ(gyro_z))
        {
            target_yaw_valid = false;
            yaw_pid.clear();
            return output;
        }

        if (!target_yaw_valid || reset_yaw_target)
        {
            target_yaw = current_yaw;
            target_yaw_valid = true;
            yaw_pid.clear();
        }

        if (absFloat(command.wz) > YAW_COMMAND_DEADBAND)
        {
            target_yaw = current_yaw;
            target_yaw_valid = true;
            yaw_pid.clear();
            output.wz = command.wz;
            return output;
        }

        const float yaw_feedback = yaw_pid.calculate(target_yaw, current_yaw);
        const float yaw_damping = YAW_GYRO_DAMPING * gyro_z;
        output.wz = limitFloat(yaw_feedback - yaw_damping,
                               -YAW_OUTPUT_MAX,
                               YAW_OUTPUT_MAX);
        return output;
    }

    ega::Chassis::Command applyLateralVelocityCompensation(
        const ega::Chassis::Command& command,
        bool active)
    {
        static bool accel_y_bias_valid = false;
        static float accel_y_bias = 0.0f;
        static float lateral_velocity = 0.0f;

        ega::Chassis::Command output = command;

        Matrixf<3, 1> accel{};
        float gyro_z = 0.0f;
        if (!ega::IMU::getPrimaryAccel(accel) ||
            !ega::IMU::getPrimaryGyroZ(gyro_z))
        {
            lateral_velocity = 0.0f;
            return output;
        }

        const float accel_y = accel(1, 0);
        const bool command_still =
            !active ||
            (absFloat(command.vx) < LATERAL_STILL_COMMAND_DEADBAND &&
             absFloat(command.vy) < LATERAL_STILL_COMMAND_DEADBAND &&
             absFloat(command.wz) < LATERAL_STILL_COMMAND_DEADBAND);

        if (command_still && absFloat(gyro_z) < LATERAL_STILL_GYRO_DEADBAND)
        {
            if (!accel_y_bias_valid)
            {
                accel_y_bias = accel_y;
                accel_y_bias_valid = true;
            }
            else
            {
                accel_y_bias =
                    accel_y_bias * (1.0f - LATERAL_BIAS_ALPHA) +
                    accel_y * LATERAL_BIAS_ALPHA;
            }
            lateral_velocity = 0.0f;
            return output;
        }

        if (!active || !accel_y_bias_valid)
        {
            lateral_velocity = 0.0f;
            return output;
        }

        float lateral_accel = accel_y - accel_y_bias;
        if (absFloat(lateral_accel) < LATERAL_ACCEL_DEADBAND)
        {
            lateral_accel = 0.0f;
        }

        lateral_velocity =
            lateral_velocity * LATERAL_VELOCITY_LEAK +
            lateral_accel * LATERAL_ACCEL_DT;

        const bool allow_compensation =
            absFloat(command.vx) > LATERAL_MIN_FORWARD_COMMAND &&
            absFloat(command.vy) < LATERAL_MAX_SIDE_COMMAND &&
            absFloat(command.wz) < LATERAL_MAX_WZ_COMMAND;

        if (!allow_compensation)
        {
            return output;
        }

        const float compensation = limitFloat(-LATERAL_COMP_GAIN * lateral_velocity,
                                              -LATERAL_COMP_MAX,
                                              LATERAL_COMP_MAX);
        output.vy = limitFloat(output.vy + compensation, -1.0f, 1.0f);
        return output;
    }

    ega::Chassis::Command applyCommandRamp(
        const ega::Chassis::Command& target,
        bool active)
    {
        static ega::Chassis::Command smooth{};

        if (!active)
        {
            smooth = {};
            return smooth;
        }

        const float velocity_delta = COMMAND_RAMP_V_RATE * COMMAND_RAMP_DT;
        const float wz_delta = COMMAND_RAMP_WZ_RATE * COMMAND_RAMP_DT;

        smooth.vx = approachFloat(smooth.vx, target.vx, velocity_delta);
        smooth.vy = approachFloat(smooth.vy, target.vy, velocity_delta);
        smooth.wz = approachFloat(smooth.wz, target.wz, wz_delta);
        return smooth;
    }
}

[[noreturn]] void RobotTask(void* pv)
{
    using namespace ega;
    constexpr TickType_t period = pdMS_TO_TICKS(5);

    RobotManager::init({
        .remote_type = RobotManager::RemoteType::DT7
    });
    MiniPC::init();
    Chassis::init(Chassis::makeDefaultConfig());

    for (;;)
    {
        TickType_t current_time = xTaskGetTickCount();

#ifdef CHASSIS_TEST_MODE
        Chassis::Command test_command{};
        constexpr TickType_t test_stage_ticks =
            CHASSIS_TEST_MOVE_TICKS + CHASSIS_TEST_STOP_TICKS;
        const TickType_t test_phase = xTaskGetTickCount() % (test_stage_ticks * 4);
        const TickType_t test_step = test_phase / test_stage_ticks;
        const TickType_t test_step_time = test_phase % test_stage_ticks;

        if (test_step_time < CHASSIS_TEST_MOVE_TICKS)
        {
            switch (test_step)
            {
            case 0:
                test_command.vx = CHASSIS_TEST_SPEED;
                break;
            case 1:
                test_command.vx = -CHASSIS_TEST_SPEED;
                break;
            case 2:
                test_command.vy = CHASSIS_TEST_SPEED;
                break;
            default:
                test_command.vy = -CHASSIS_TEST_SPEED;
                break;
            }
        }
        // test_command.vx = 0.80f;
        // test_command.vy = 0.0f;
        // test_command.wz = 0.0f;
        Chassis::setMode(Chassis::Mode::OpenLoop);
        const auto assisted_command = test_command;
        const auto yaw_command =
            applyYawTargetControl(assisted_command, true);
        Chassis::setCommand(applyCommandRamp(yaw_command, true));
#else
        RobotManager::update();
        const auto robot_command = RobotManager::getCommand();
        const auto minipc_command = MiniPC::getCommand();
        const auto* dt7 = getDT7Remote();

        bool chassis_enable = false;
        bool reset_yaw_target = false;
        Chassis::Command chassis_command{};

        if (dt7 != nullptr && dt7->isOnline() && dt7->isFrameValid())
        {
            const auto right_switch = dt7->getRightSwitch();
            const auto left_switch = dt7->getLeftSwitch();

            if (right_switch == DT7::RCSwitchState::MIDDLE &&
                left_switch == DT7::RCSwitchState::MIDDLE &&
                robot_command.chassis.enable)
            {
                chassis_enable = true;
                chassis_command = {
                    .vx = robot_command.chassis.vx,
                    .vy = robot_command.chassis.vy,
                    .wz = robot_command.chassis.wz,
                };
            }
            else if (right_switch == DT7::RCSwitchState::MIDDLE &&
                     left_switch == DT7::RCSwitchState::UP &&
                     minipc_command.enable)
            {
                chassis_enable = true;
                reset_yaw_target = minipc_command.reset_yaw_target;
                chassis_command = toChassisCommand(minipc_command);
            }
        }
        else if (minipc_command.enable)
        {
            chassis_enable = true;
            reset_yaw_target = minipc_command.reset_yaw_target;
            chassis_command = toChassisCommand(minipc_command);
        }

        if (chassis_enable)
        {
            Chassis::setMode(Chassis::Mode::OpenLoop);
            const auto assisted_command = chassis_command;
            const auto yaw_command =
                applyYawTargetControl(assisted_command,
                                      true,
                                      reset_yaw_target);
            Chassis::setCommand(applyCommandRamp(yaw_command, true));
        }
        else
        {
            (void)applyCommandRamp({}, false);
            (void)applyYawTargetControl({}, false);
            Chassis::setMode(Chassis::Mode::Protect);
        }
#endif

        Chassis::update();
        Chassis::debug_printf();

        vTaskDelayUntil(&current_time, period / portTICK_RATE_MS);
    }
}
