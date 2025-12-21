//
// Created by zhangzhiwen on 25-12-21.
//
#include "main.h"
#include "task_manager.h"

int main()
{
	// Init cubeMX config
	MXMainInit();

	Task_Init();

	while (true)
	{
		HAL_Delay(10);
	}
}
