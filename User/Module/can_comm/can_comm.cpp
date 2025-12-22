//
// Created by Hrmys3 on 2025/11/4.
//

#include "can_comm.h"

// ---
// 注意：
// 由于 CanComm 是一个模板类，其所有成员函数的实现
// (如构造函数, send(), driver_rx_callback())
// 都必须被编译器在编译时看到，因此它们必须被定义在头文件 "can_comm.h" 中。
//
// 如果未来有 CanComm 类的非模板化静态成员变量，也应在这里定义。
// ---
