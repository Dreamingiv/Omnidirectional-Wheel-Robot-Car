//
// Created by Hrmys3 on 2025/10/19.
//

#include "driver_usb.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"
#include <cstring>

// 引入 ST 库中需要访问的全局变量 (在 usbd_cdc_if.c 和 usb_device.c 中定义)
extern "C" {
// 在 usbd_cdc_if.c 中定义的 CDC 接口操作结构体
// 我们将通过修改它的成员来实现 Hook
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

// 在 usbd_cdc_if.c 中定义的接收缓冲区
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

// 在 usb_device.c 中定义的 USB 设备核心句柄
extern USBD_HandleTypeDef hUsbDeviceFS;
}

namespace ega
{
    /**
     * @brief USB 类的实现
     */

    /**
     * @brief 获取单例实例
     */
    USB& USB::getInstance()
    {
        // C++11 保证了静态局部变量的初始化是线程安全的
        static USB instance;
        return instance;
    }

    /**
     * @brief 构造函数, 初始化成员变量
     */
    USB::USB() :
        rx_buffer_(nullptr), tx_callback_user_(nullptr), rx_callback_user_(nullptr)
    {
        // 初始化时保持简洁
    }

    /**
     * @brief 初始化 USB, 注册回调并获取接收缓冲区
     */
    void USB::init(const Config& config)
    {
        // 1. 保存回调
        getInstance().tx_callback_user_ = config.tx_callback;
        getInstance().rx_callback_user_ = config.rx_callback;

        // 2. 返回接收缓冲区指针
        getInstance().rx_buffer_ = UserRxBufferFS;

        // 3. 【核心 Hook】重定向 ST 库中的函数指针
        // 这将覆盖 usbd_cdc_if.c 中 USBD_Interface_fops_HS.Receive 和 .TransmitCplt 的默认值
        USBD_Interface_fops_FS.Receive = USBReceiveCallback;
        USBD_Interface_fops_FS.TransmitCplt = USBTransmitCallback;

    }

    /**
     * @brief 通过 USB (CDC) 发送数据
     */
    uint8_t USB::send(uint8_t* buf, uint16_t len)
    {
        // 调用 C 语言实现的 USB 发送函数
        //logger_printf("transmitting\r\n");
        uint8_t result = CDC_Transmit_FS(buf, len);
        return result;
    }


}

// --------------------------------------------------------------------
// 静态 Hook 函数实现 (extern "C" 是隐式的，因为它们是类静态成员)
// --------------------------------------------------------------------
/**
 * @brief 接收数据 Hook，在 USB 收到 OUT 包时被 ST 库调用
 * @note STM32 USB中断服务程序自动调用USB::Receive()
 */
int8_t USBReceiveCallback(uint8_t* buf, uint32_t* len)
{
    using namespace ega;
    // 1. 复刻 usbd_cdc_if.c 中 CDC_Receive_HS 的原始功能
    //设置USB接收缓冲区
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &buf[0]);
    //重新激活USB OUT端点接收数据包，准备接收下一个数据包
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);

    // 2. 触发用户注册的接收回调
    USB& instance = USB::getInstance();
    instance.rx_len_ = (uint16_t)*len;
    if (instance.rx_callback_user_ != nullptr)
    {
        instance.rx_callback_user_(buf, (uint16_t)*len);
    }

    return (USBD_OK);
}

/**
 * @brief 发送完成 Hook，在 USB IN 包发送完成后被 ST 库调用
 */
int8_t USBTransmitCallback(uint8_t* buf, uint32_t* len, uint8_t epnum)
{
    using namespace ega;
    // 1. 复刻 usbd_cdc_if.c 中 CDC_TransmitCplt_HS 的原始功能 (通常是 UNUSED)
    (void)buf;
    (void)len;
    (void)epnum;

    // 2. 触发用户注册的发送完成回调
    USB& instance = USB::getInstance();
    if (instance.tx_callback_user_ != nullptr)
    {
        instance.tx_callback_user_();
    }

    return (USBD_OK);
}
