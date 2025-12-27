#ifndef MODULE_CAN_COMM_H_
#define MODULE_CAN_COMM_H_

#include "driver_can.h"
#include "crc8.h"
#include <cstdint>
#include <functional>
#include <cstring>

/**
 * @brief CAN 通用通信模板类
 * * 用于在 CAN 总线上发送和接收定长结构体。
 * 包含自动分包、CRC校验和拼包逻辑。
 * * 帧结构（Header + Data）：
 * | Byte | 内容 |
 * |------|------|
 * | 0    | 'E' (Magic) |
 * | 1-2  | Data Length (LE) |
 * | 3    | CRC8 |
 * | 4-7  | Data[0..3] |
 * | 后续帧 | Data[4..] 每帧8字节 |
 * * @tparam T 需要传输的结构体类型
 */

namespace ega
{
    template <typename T>
    class CanComm
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        /**
         * @brief 协议相关常量定义
         */
        struct Protocol
        {
            static constexpr uint8_t HEADER_PACKET_MAGIC = 'E'; ///< 帧头魔数
            static constexpr size_t HEADER_PACKET_LEN = 8; ///< 头帧固定长度
            static constexpr size_t FRAME_PAYLOAD_LEN = 8; ///< 后续数据帧载荷长度
            static constexpr size_t MAX_RX_BUFFER_LEN = 80; ///< 最大接收缓存大小
        };

        /* ====================== 2. 内部类型定义 ====================== */
        /**
         * @brief 接收状态机枚举
         */
        enum class RxState
        {
            IDLE, ///< 空闲状态，等待 Header
            RECEIVING ///< 正在接收后续数据包
        };

        /**
         * @brief 接收回调函数类型定义
         */
        using RxCallback = std::function<void(const T& received_struct)>;

        /**
         * @brief 配置结构体
         */
        struct Config
        {
            CAN_HandleTypeDef* handle = nullptr; ///< CAN 句柄
            uint32_t tx_id = 0; ///< 发送 ID
            uint32_t rx_id = 0; ///< 接收 ID
            RxCallback rx_callback = nullptr; ///< 接收完成后的回调
        };

        /* ====================== 3. 静态接口 ====================== */

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        explicit CanComm(const Config& config);
        ~CanComm() = default;

        // 禁止拷贝构造和赋值，防止资源竞争或逻辑错误
        CanComm(const CanComm&) = delete;
        CanComm& operator=(const CanComm&) = delete;
        /* ====================== 5. 公共接口 ====================== */
    public:
        /**
         * @brief 发送结构体数据
         * * 将结构体拆分为 Header 帧和若干数据帧发送。
         * @param data 待发送的结构体引用
         * @return true 发送成功
         * @return false 发送失败（底层发送失败）
         */
        bool send(const T& data);

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        CANInstance can_instance_; ///< 内部 CAN 驱动实例
        //uint32_t tx_id_; ///< 应用层 TX ID（备份） //疑似没用
        //uint32_t rx_id_;
        RxCallback rx_callback_; ///< 应用层接收回调
        RxState rx_state_; ///< 当前接收状态

        uint8_t rx_buffer_[Protocol::MAX_RX_BUFFER_LEN]{}; ///< 接收缓冲区（静态分配）

        uint16_t expected_size_ = 0; ///< 期望接收的总字节数（从 Header 解析）
        uint8_t expected_crc_ = 0; ///< 期望的 CRC8（从 Header 解析）
        uint16_t received_bytes_ = 0; ///< 当前已接收字节数

    private:
        /**
         * @brief 底层 CAN 驱动接收回调
         * * 处理分包重组、状态机流转及 CRC 校验。
         * @param data 接收到的原始字节数组
         * @param len 数据长度
         */
        void driver_rx_callback(const uint8_t* data, uint8_t len);
    };


    // =============================================================================
    // 模板类实现部分 (Template Implementation)
    // =============================================================================

    template <typename T>
    CanComm<T>::CanComm(const Config& config)
        : //tx_id_(config.tx_id),rx_id_(config.rx_id),
        can_instance_({
            .handle = config.handle,
            .tx_id = config.tx_id,
            .rx_id = config.rx_id,
            // ✅ 使用 lambda 捕获 this，将底层 C 风格回调转发给成员函数
            .rx_callback = [this](const uint8_t* data, uint8_t len)
            {
                driver_rx_callback(data, len);
            }
        }),
        rx_callback_(config.rx_callback),
        rx_state_(RxState::IDLE)
    {
        memset(rx_buffer_, 0, sizeof(rx_buffer_));
    }

    template <typename T>
    bool CanComm<T>::send(const T& data)
    {
        const auto* data_ptr = reinterpret_cast<const uint8_t*>(&data);
        const auto total_size = static_cast<uint16_t>(sizeof(T));
        const uint8_t crc = crc_8(data_ptr, total_size);

        // 1️⃣ 组装 Header 帧 (包含 Magic, Size, CRC 以及前4字节数据)
        CANInstance::msg_t header_msg{};
        header_msg.length = static_cast<uint8_t>(Protocol::HEADER_PACKET_LEN);

        // Header 头部信息
        header_msg.data[0] = Protocol::HEADER_PACKET_MAGIC;
        header_msg.data[1] = static_cast<uint8_t>(total_size & 0xFF);
        header_msg.data[2] = static_cast<uint8_t>((total_size >> 8) & 0xFF);
        header_msg.data[3] = crc;

        // Header 数据部分（最多携带首4字节）
        uint8_t first_data_len = (total_size >= 4) ? 4 : total_size;
        memcpy(&header_msg.data[4], data_ptr, first_data_len);

        if (!can_instance_.send(header_msg))
        {
            return false;
        }

        // 2️⃣ 发送剩余数据帧
        uint16_t bytes_sent = first_data_len;
        while (bytes_sent < total_size)
        {
            CANInstance::msg_t data_msg{};

            // 计算当前帧载荷长度
            uint8_t payload_len =
                (total_size - bytes_sent > Protocol::FRAME_PAYLOAD_LEN)
                    ? static_cast<uint8_t>(Protocol::FRAME_PAYLOAD_LEN)
                    : static_cast<uint8_t>(total_size - bytes_sent);

            memcpy(data_msg.data, data_ptr + bytes_sent, payload_len);
            data_msg.length = payload_len;

            if (!can_instance_.send(data_msg))
            {
                return false;
            }

            bytes_sent += payload_len;
        }
        return true;
    }

    template <typename T>
    void CanComm<T>::driver_rx_callback(const uint8_t* data, uint8_t len)
    {
        if (len == 0) return;

        // 1️⃣ 处理 Header 帧 (检测 Magic 和 长度)
        if (data[0] == Protocol::HEADER_PACKET_MAGIC && len == Protocol::HEADER_PACKET_LEN)
        {
            // 解析 Header 信息
            expected_size_ = (static_cast<uint16_t>(data[2]) << 8) | data[1];
            expected_crc_ = data[3];
            received_bytes_ = 0;

            // 安全性检查：大小是否匹配本地结构体，是否超出缓存
            if (expected_size_ != sizeof(T) || expected_size_ > Protocol::MAX_RX_BUFFER_LEN)
            {
                rx_state_ = RxState::IDLE;
                return;
            }

            // 提取 Header 中的首段数据
            uint8_t first_data_len = (expected_size_ >= 4) ? 4 : expected_size_;
            memcpy(rx_buffer_, &data[4], first_data_len);
            received_bytes_ = first_data_len;

            rx_state_ = RxState::RECEIVING;

            // ⭐ 特殊情况：如果数据较小，在 Header 里已经全部接收完毕
            if (received_bytes_ == expected_size_)
            {
                uint8_t calculated_crc = crc_8(rx_buffer_, expected_size_);
                if (calculated_crc == expected_crc_)
                {
                    if (rx_callback_)
                    {
                        rx_callback_(*reinterpret_cast<T*>(rx_buffer_));
                    }
                }
                rx_state_ = RxState::IDLE;
            }
            return;
        }

        // 2️⃣ 处理后续数据帧
        if (rx_state_ != RxState::RECEIVING)
        {
            return;
        }

        uint8_t payload_len = len;

        // 基本合法性检查
        if (payload_len == 0 || payload_len > Protocol::FRAME_PAYLOAD_LEN)
        {
            rx_state_ = RxState::IDLE;
            return;
        }
        // 防止缓冲区溢出
        if (received_bytes_ + payload_len > expected_size_)
        {
            rx_state_ = RxState::IDLE;
            return;
        }

        // 拼接到接收缓冲区
        memcpy(rx_buffer_ + received_bytes_, data, payload_len);
        received_bytes_ += payload_len;

        // 3️⃣ 接收完毕 -> CRC 校验 -> 回调
        if (received_bytes_ == expected_size_)
        {
            uint8_t calculated_crc = crc_8(rx_buffer_, expected_size_);
            if (calculated_crc == expected_crc_)
            {
                if (rx_callback_)
                {
                    rx_callback_(*reinterpret_cast<T*>(rx_buffer_));
                }
            }
            // 无论校验成功与否，重置状态
            rx_state_ = RxState::IDLE;
        }
    }
}
#endif  //MODULE_CAN_COMM_H_
