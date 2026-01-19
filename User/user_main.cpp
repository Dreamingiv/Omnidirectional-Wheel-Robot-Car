#include "main.h"

#include "task_manager.h"


/**
 * @brief 主函数
 * @details 初始化系统，配置外设，创建FreeRTOS任务，进入主循环
 * @attention 使用宏定义hook了cubemx生成的main函数
 */
int main() {
    /* 使用宏定义hook了main.c中的main函数 */
    MX_Main_Init();

	/* 初始化FreeRTOS任务 */
	Task_Init();

	/* 正常来说代码不会进到这里 */
    Error_Handler();
}
