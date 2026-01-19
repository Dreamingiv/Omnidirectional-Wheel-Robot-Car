//
// Author: Breezeee
// Date: 25-5-28
//

// =============================== 引入头文件 ===============================
#include <cstdint>
#include "memory.h"
#include "usart.h"
#include "cmsis_os.h"

#include "driver_dwt.h"
#include "driver_usart.h"
#include "driver_can.h"
#include "driver_usb.h"
#include "driver_gpio.h"
#include "driver_pwm.h"

#include "can_comm.h"
#include "daemon.h"
#include "logger.h"
#include "remote_DT7.h"
#include "remote_FS_I6X.h"
#include "remote_VT13.h"

#include "task_manager.h"
#include "daemon_task.h"
#include "motor_task.h"
#include "imu_task.h"
#include "robot_task.h"

#include "PID.h"
#include "SMC.h"
#include "buzzer_on_board.h"
#include "key_on_board.h"
#include "led_on_board.h"
#include "music.h"
#include "servo.h"

#include "motor.h"
#include "DJIMotor.h"
#include "DMMotor.h"
#include "LKMotor.h"


// 默认任务句柄
TaskHandle_t default_task_handle;

/**
 * @brief  初始化用户编写的外设
 * @note 	 主要写框架内的设备。HAL库的不要写在这里
 */
void BSP_Init()
{
    using namespace ega;
    /* 用户初始化内容，在进入默认任务之后被调用 */
    DWTInstance::init(); //用于计时，默认480MHz
    Logger::init(); //用于输出日志，默认使用串口1
    LEDOnBoard::init();
    BuzzerOnBoard::init();
    KeyOnBoard::init();
    USB::init();
}

/**
 * @brief  创建系统任务
 * @note   使用 FreeRTOS 的 xTaskCreate 注册默认任务入口。
 */
void Task_Init()
{
    // 创建默认任务
    BaseType_t xReturn;
    xReturn = xTaskCreate(DefaultTask,
                          "默认任务",
                          2048,
                          NULL,
                          7,
                          &default_task_handle);
    if (pdFALSE != xReturn)
    {
        vTaskStartScheduler();
    }
}

/**
 * @brief  默认任务，在此任务中注册所有实际运行的任务，注册完成后自毁
 * @param  pv: 任务参数（当前未使用）
 * @note
 */

void DefaultTask(void* pv)
{
    /* HAL库初始化函数，部分设备初始化必须发生在调度器启动后 */
    MX_USB_DEVICE_Init();

    /* 用户初始化函数 */
    BSP_Init();

    /* 创建一系列任务 */
    BaseType_t xReturn = pdPASS;

    taskENTER_CRITICAL(); //创建任务时关闭中断
    //判断任务是否创建成功，一旦存在创建不成功的任务，xReturn置0
    xReturn &= xTaskCreate(
        DaemonTask,
        "守护进程",
        512,
        NULL,
        6,
        NULL) == pdPASS;

    xReturn &= xTaskCreate(
        IMUTask,
        "陀螺仪",
        2048, //必须够大，小了会卡死
        NULL,
        5,
        NULL) == pdPASS;

    xReturn &= xTaskCreate(
        MotorTask,
        "电机控制",
        2048,
        NULL,
        4,
        NULL) == pdPASS;

    xReturn &= xTaskCreate(
        RobotTask,
        "机器人控制任务",
        4096,
        NULL,
        4,
        NULL) == pdPASS;

    xReturn &= xTaskCreate(
        TestTask,
        "测试任务",
        2048,
        NULL,
        4,
        NULL) == pdPASS;
    taskEXIT_CRITICAL();

    //如果存在任务没有被分配成功，直接进入死循环
    if (xReturn != pdPASS)
    {
        Error_Handler();
    }
    vTaskDelete(default_task_handle);
}

/**
 * @brief  测试任务
 * @param  pv: 任务参数（当前未使用）
 * @note   当前作为测试任务
 */

void TestTask(void* pv)
{
    using namespace ega;
    constexpr TickType_t period = pdMS_TO_TICKS(10);

    DJIMotor dji_motor({
        .type = DJIMotor::Type::GM6020,
        .direction = Motor::Direction::REVERSE,
        .can_handle = &hcan2,
        .motor_id = 1,
        // .reduction_radio =
    });

    DMMotor dm_motor({
    .direction = Motor::Direction::NORMAL,
    .can_handle = &hcan1,
    .can_tx_id = 0x02,
    .can_rx_id = 0x12,
    .p_max_abs = 12.5,
    .v_max_abs = 30,
    .t_max_abs = 10,
    // .external_reduction_radio = ,
    // .use_mit =
    });

    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;)
    {
        // logger_printf("Hello world\r\n");
        // dji_motor.setEffort(-1);
        // dm_motor.setEffort(1);
        auto measure = dm_motor.getMeasure();
        auto dm_measure = dm_motor.getDMMeasure();
        logger_printf("%f,%f，%d\r\n",measure.total_angle,measure.torque,dm_measure.state);

        LEDOnBoard::loop();

        vTaskDelayUntil(&last_wake_time, period);
    }
}
