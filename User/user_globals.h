#ifndef GLOBALS_H
#define GLOBALS_H

namespace ega::globals
{
  /**
   * @brief 全局变量的统一存放点
   * @note
   *  - 用于在不同任务 / 模块之间共享数据，通过 ega::globals::var 访问
   *  - 该变量使用 C++17 inline 变量定义，可直接在头文件中定义并包含于多个翻译单元
   *  - inline 变量本身会默认初始化，但如果有自己写的类，建议显式初始化
   *  - 使用前请确保工程以 C++17 或以上标准编译
   */
inline int var;
}



#endif //GLOBALS_H
