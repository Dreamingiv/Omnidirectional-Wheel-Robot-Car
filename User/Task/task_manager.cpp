//
// Created by zhangzhiwen on 25-12-21.
//

#include "task_manager.h"
#include "main.h"
#include "usb_device.h"
#include "FreeRTOS.h"
#include "task.h"

// =============================== 引入头文件 ===============================
#include "driver_dwt.h"
#include "driver_usb.h"
#include "key_on_board.h"
#include "led_on_board.h"
#include "logger.h"


// 默认任务句柄
static TaskHandle_t default_task_handle;

// =============================== 静态创建任务参数 ===========================
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
	Logger::init(); //用于输出日志，默认使用串口1
	LEDOnBoard::init();
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
	// xReturn &= xTaskCreate(
	//     DaemonTask,
	//     "守护进程",
	//     512,
	//     NULL,
	//     6,
	//     NULL) == pdPASS;
	//
	// xReturn &= xTaskCreate(
	//     IMUTask,
	//     "陀螺仪",
	//     2048, //必须够大，小了会卡死
	//     NULL,
	//     5,
	//     NULL) == pdPASS;
	//
	// xReturn &= xTaskCreate(
	//     MotorTask,
	//     "电机控制",
	//     2048,
	//     NULL,
	//     4,
	//     NULL) == pdPASS;
	//
	// xReturn &= xTaskCreate(
	//     RobotTask,
	//     "机器人控制任务",
	//     4096,
	//     NULL,
	//     4,
	//     NULL) == pdPASS;
	//
	// xReturn &= xTaskCreate(
	// 	TestTask,
	// 	"测试任务",
	// 	2048,
	// 	NULL,
	// 	4,
	// 	NULL) == pdPASS;
	test_task_handle_ = xTaskCreateStatic(
		TestTask,
		"TestTask",
		1024,
		nullptr,
		1,
		test_stack_buffer_,
		&test_task_buffer_);
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
	for (;;)
	{
		static size_t cnt = 0;
		logger_printf("%d\r\n", cnt++);
		ega::LEDOnBoard::loop();
		vTaskDelay(5);
	}
}
