//
// Created by zhangzhiwen on 25-12-21.
//

#ifndef DRIVER_CAN_H
#define DRIVER_CAN_H

#include "main.h"
#include "can.h"
#include <array>
#include <functional>

namespace ega
{
    class CANInstance
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        friend void ::HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan);
        static constexpr size_t MX_INS_NUM = 14; // 单条总线最大负载
        static constexpr size_t MX_DEV_NUM = 2; // 芯片支持的CAN外设数量，对于F407来说有2个CAN外设
        static constexpr size_t MX_MSG_LEN = 8; // 单帧最大数据长度,单位字节

        using RxCallback = std::function<void(const uint8_t* data, uint8_t len)>;
        using TxCallback = std::function<void()>;

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        struct msg_t
        {
            uint8_t data[MX_MSG_LEN] = {0};
            uint8_t length = MX_MSG_LEN; // 默认情况下为8，必要时可自行修改
        };

        struct Config
        {
            CAN_HandleTypeDef* handle = nullptr; // can句柄
            uint32_t tx_id = 0;
            TxCallback tx_callback = nullptr; // 不提供。 日后需要写发送回调时才有用
            uint32_t rx_id = 0;
            RxCallback rx_callback = nullptr; // 回调函数：参数 (id, data, len)
        };

        /* ====================== 3. 静态接口 ====================== */
    private:
        // 回调设置,需要在外部调用所以设置为static
        static void RxFifoCallback(CAN_HandleTypeDef* hfdcan, uint32_t fifo);

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        CANInstance(const Config& config);
        ~CANInstance();

        /* ====================== 5. 公共接口 ====================== */
    public:
        void setDataLength(uint8_t length);
        bool send(const msg_t& msg, uint16_t block_timeout_us = 150); // 如果FIFO满，允许超时150us
        // 测试用的方法，严格来说该方法会破坏实例的封装性
        bool send(uint32_t target_id, const msg_t& msg, uint16_t block_timeout_us = 300);

        [[nodiscard]] CAN_HandleTypeDef* getHandle() const { return handle_; }
        uint8_t getHandleIndex() const { return getHandle() == &hcan1 ? 0 : getHandle() == &hcan2 ? 1 : 2; }

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        // 静态共享指针数组,用于存储所有CAN实例,用于中断回调函数
        // 每个 FDCAN 外设一组定长表 + 计数，避免运行期堆分配
        static std::array<CANInstance*, MX_INS_NUM> instance_[MX_DEV_NUM];
        static uint8_t instance_count_[MX_DEV_NUM];
        // 用于分配过滤器,每次添加新的实例时,会分配一个过滤器并递增此变量
        static uint8_t can1_filter_idx_, can2_filter_idx_, can3_filter_idx_;

        // 每个instance的私有成员
        CAN_HandleTypeDef* handle_; // can句柄
        // 发送时自动设置
        CAN_TxHeaderTypeDef tx_header_{}; // CAN报文发送配置

        // tx
        uint32_t tx_id_; // 发送id
        uint8_t tx_len_{}; // 疑似没什么用，这里假定发送的数据都是8位
        TxCallback tx_callback_; // 传输完成回调函数
        uint8_t tx_buff_[MX_MSG_LEN]{}; // 发送缓存

        // rx
        uint32_t rx_id_; // 接收id
        uint8_t rx_buff_[MX_MSG_LEN]{}; // 接收缓存,最大消息长度为8
        uint8_t rx_len_{}; // 接收长度,可能为0-8
        RxCallback rx_callback_;

    private:
        // 构建实例时为接收ID设置过滤规则
        void addFilter();
        // 启动CAN服务和接收中断
        static void serviceInit();
    };
} // namespace ega










#endif //DRIVER_CAN_H
