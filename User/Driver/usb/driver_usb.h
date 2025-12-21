//
// Created by Hrmys3 on 2025/10/19.
//

#ifndef DRIVER_USB_H_
#define DRIVER_USB_H_


#include <cstdint>
#include <functional>

/**
 * @file driver_usb.hpp
 * @brief C++ 封装的 USB VCP (Virtual COM Port) 驱动
 * @note 采用单例模式
 * @attention 依赖于 ST USB 中间件和在 usbd_cdc_if.c 中实现的
 * CDCInitRxbufferNcallback() 和 CDC_Transmit_HS()
 */

// 告诉 C++ 编译器, 这些头文件中的函数和类型是 C 语言的
extern "C" {
#include "usbd_def.h"
#include "usbd_cdc.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"
}

// 声明为extern "C"的独立函数，确保与ST库兼容
extern "C" {
int8_t USBReceiveCallback(uint8_t* buf, uint32_t* len);
int8_t USBTransmitCallback(uint8_t* buf, uint32_t* len, uint8_t epnum);
}

namespace ega
{
    /**
     * @class USB
     * @brief USB 虚拟串口 (CDC) 的 C++ 封装类
     */
    class USB
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        /**
         * @brief 替换 usbd_cdc_if.c 中的 CDC_Receive_HS/FS 的 Hook
         */
        friend int8_t (::USBReceiveCallback)(uint8_t* buf, uint32_t* len);
        /**
         * @brief 替换 usbd_cdc_if.c 中的 CDC_TransmitCplt_HS/FS 的 Hook
         */
        friend int8_t (::USBTransmitCallback)(uint8_t* buf, uint32_t* len, uint8_t epnum);
        using TxCallback = std::function<void()>;
        using RxCallback = std::function<void(uint8_t* data, uint16_t len)>;

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        struct Config
        {
            TxCallback tx_callback = nullptr;
            RxCallback rx_callback = nullptr;
        };

        /* ====================== 3. 静态接口 ====================== */
    public:
        /**
         * @brief 获取 USB 的单例实例
         * @return USB& 类的唯一实例的引用
         */
        static USB& getInstance();
        /**
         * @brief 初始化 USB, 注册回调并获取接收缓冲区
         * @param config 包含 tx/rx 回调函数的配置结构体
         * @attention 请在FreeRTOS完全启动后再初始化USB类，否则可能造成FreeRTOS定时器异常
         */
        static void init(const Config& config = {
            .tx_callback = nullptr,
            .rx_callback = nullptr,
        });
        /**
         * @brief 通过 USB (CDC) 发送数据
         * @param buf 要发送的数据缓冲区指针
         * @param len 要发送的数据长度
         */
        static uint8_t send(uint8_t* buf, uint16_t len);
        /**
         * @brief 获取接收缓冲区的指针 (封装了对内部成员的访问)
         * @return uint8_t* 指向接收缓冲区的指针
         */
        static uint8_t* getRxBuffer() { return getInstance().rx_buffer_; };
        /**
         * @brief 获取上一次收到的消息长度
         * @return uint16_t 消息长度
         */
        static uint16_t getRxLen() { return getInstance().rx_len_; };

        /* ====================== 4. 构造 / 析构 ====================== */
    private:
        USB();
        ~USB() = default;
    public:
        // 删除拷贝构造和赋值操作, 确保单例的唯一性
        USB(const USB&) = delete;
        USB& operator=(const USB&) = delete;

        /* ====================== 5. 公共接口 ====================== */

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        uint8_t* rx_buffer_;
        uint16_t rx_len_{};
        TxCallback tx_callback_user_; // 保存用户注册的回调
        RxCallback rx_callback_user_; // 保存用户注册的回调
    };
}
#endif // DRIVER_USB_H_
