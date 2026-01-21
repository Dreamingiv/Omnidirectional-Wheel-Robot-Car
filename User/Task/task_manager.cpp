//
// Created by zhangzhiwen on 25-12-21.
//

#include "task_manager.h"
#include "main.h"
#include "usb_device.h"
#include "FreeRTOS.h"
#include "task.h"

// =============================== 引入头文件 ===============================
#include "buzzer_on_board.h"
#include "can_comm.h"
#include "daemon_task.h"
#include "DJIMotor.h"
#include "DMMotor.h"
#include "driver_dwt.h"
#include "driver_usb.h"
#include "imu_task.h"
#include "key_on_board.h"
#include "led_on_board.h"
#include "logger.h"
#include "motor_task.h"
#include "PID.h"
#include "robot_task.h"

using namespace ega;

// 默认任务句柄
static TaskHandle_t default_task_handle;

// =============================== 静态创建任务参数 ===========================
static TaskHandle_t daemon_task_handle_;
static StackType_t  daemon_stack_buffer_[512];
static StaticTask_t daemon_task_buffer_;

static TaskHandle_t imu_task_handle_;
static StackType_t  imu_stack_buffer_[2048];
static StaticTask_t imu_task_buffer_;

static TaskHandle_t motor_task_handle_;
static StackType_t  motor_stack_buffer_[2048];
static StaticTask_t motor_task_buffer_;

static TaskHandle_t robot_task_handle_;
static StackType_t  robot_stack_buffer_[4096];
static StaticTask_t robot_task_buffer_;

static TaskHandle_t test_task_handle_;
static StackType_t  test_stack_buffer_[1024];
static StaticTask_t test_task_buffer_;

/**
 * @brief  初始化用户编写的外设
 * @note 	 主要写框架内的设备。HAL库的不要写在这里
 */
void BSP_Init()
{
	using namespace ega;
	// /* 用户初始化内容，在进入默认任务之后被调用 */
	DWTInstance::init(); //用于计时，默认168MHz
	Logger::init();      //用于输出日志，默认使用串口1
	LEDOnBoard::init();
	KeyOnBoard::init();
	USB::init();
	BuzzerOnBoard::init();
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

	daemon_task_handle_ = xTaskCreateStatic(
		DaemonTask,
		"DaemonTask",
		512,
		nullptr,
		6,
		daemon_stack_buffer_,
		&daemon_task_buffer_);

	imu_task_handle_ = xTaskCreateStatic(
		IMUTask,
		"IMUTask",
		2048,
		nullptr,
		5,
		imu_stack_buffer_,
		&imu_task_buffer_);

	motor_task_handle_ = xTaskCreateStatic(
		MotorTask,
		"MotorTask",
		2048,
		nullptr,
		5,
		motor_stack_buffer_,
		&motor_task_buffer_);
	//
	// xReturn &= xTaskCreate(
	//     RobotTask,
	//     "机器人控制任务",
	//     4096,
	//     NULL,
	//     4,
	//     NULL) == pdPASS;
	//
	robot_task_handle_ = xTaskCreateStatic(
		RobotTask,
		"RobotTask",
		4096,
		nullptr,
		4,
		robot_stack_buffer_,
		&robot_task_buffer_);

	test_task_handle_ = xTaskCreateStatic(
		TestTask,
		"TestTask",
		1024,
		nullptr,
		4,
		test_stack_buffer_,
		&test_task_buffer_);

	taskEXIT_CRITICAL();

	//如果存在任务没有被分配成功，直接进入死循环
	// if (xReturn != pdPASS)
	// {
	// 	Error_Handler();
	// }
	vTaskDelete(default_task_handle);
}

/**
 * @brief  测试任务
 * @param  pv: 任务参数（当前未使用）
 * @note   当前作为测试任务
 */

void TestTask(void* pv)
{
	// ega::DJIMotor dji_motor=ega::DJIMotor({
	// 	.type =ega::DJIMotor::Type::GM6020,
	// 	.can_handle = &hcan1,
	// 	.motor_id = 5
	//  });
	//
	// Motor::Measure measure=Motor::Measure();
	//
	// PID pid_controlller=PID({
	// 	.kp=5,
	// 	.ki=0,
	// 	.kd=1,
	// 	.limit_output = 10
	// });

	DMMotor dm_motor=DMMotor({
	.can_handle = &hcan1,
		.can_tx_id = 0x00,
		.can_rx_id = 0x01,
		.p_max_abs = 3,
		.v_max_abs = 3,
		.t_max_abs = 0.1
	});

	for (;;)
	{

		// static size_t cnt = 0;
		// logger_printf("%d\r\n", cnt++);

		// static float output=0;
		// measure=dji_motor.getMeasure();
		// output=pid_controlller.calculate(20,measure.total_angle);
		// dji_motor.setEffort(output);
		// logger_printf("%f\r\n",measure.total_angle);



		ega::LEDOnBoard::loop();
		vTaskDelay(5);
	}
}
