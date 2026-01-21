//
// Created by An on 2025/10/24.
//
#include "shoot.h"
#include "logger.h"
#include "robot_task.h"
#include "motor.h"
#include "imu_task.h"

//void Shoot::controlLoop() {
//
//    const auto shoot_state = Remote::getInstance().getShootState();
//
//    if (pluck_wheel->getMeasure().total_position) {
//        if (Init_flag == INIT_CONFIG::ENABLE) {
//            pluck_wheel_target = pluck_wheel->getMeasure().total_position;
//            Init_flag = INIT_CONFIG::DISABLE;
//        }
//    }
//
//    if (shoot_state.fric_mode == FricMode::ON) {
//        fric_left->setRef(fric_speed);
//        fric_right->setRef(fric_speed);
//    }else {
//        fric_left->setRef(0);
//        fric_right->setRef(0);
//    }
//
//    if (shoot_state.rammer_permission == RammerPermission::PERMIT) {
//        if (fabs(pluck_wheel_target - pluck_wheel->getMeasure().total_position) < dead_zone) {
//            if (shoot_state.rammer_channel == dial_set) {
//                pluck_wheel_target += pluck_step_size;
//            }
//        }
//    }
//
//    pluck_wheel->setRef(pluck_wheel_target);
//
//}


// Shoot::Shoot(const Config &config) {
//     fric_left = Motor::create(config.Fric_left_control);
//     fric_right = Motor::create(config.Fric_right_control);
//     pluck_wheel = Motor::create(config.Pluck_wheel_control);
//     fric_speed = config.fric_speed_control;
//     pluck_step_size = config.pluck_step_size_control;
//     dead_zone = config.dead_zone_control;
// }
#include "logger.h"
#include "robot_task.h"
#include "motor.h"
#include "imu_task.h"

