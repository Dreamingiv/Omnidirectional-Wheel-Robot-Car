//
// Created by zhangzhiwen on 25-12-21.
//

#include "driver_can.h"
#include "memory.h"
#include "driver_dwt.h"

namespace ega
{
	std::array<CANInstance*, CANInstance::MX_INS_NUM> CANInstance::instance_[MX_DEV_NUM]{};

	uint8_t CANInstance::instance_count_[MX_DEV_NUM] = {0, 0};
	uint8_t CANInstance::can1_filter_idx_            = 0;
	uint8_t CANInstance::can2_filter_idx_            = 0;
	uint8_t CANInstance::can3_filter_idx_            = 0;

	CANInstance::CANInstance(const Config& config)
	{
		//config合法性检查
		if (config.handle == nullptr)
		{
			//等到添加日志模块后，写报错信息
			Error_Handler();
		}
		if (config.tx_id > 0x7FF || config.rx_id > 0x7FF || config.tx_id == config.rx_id || !config.tx_id || !config.
			rx_id)
		{
			//等到添加日志模块后，写报错信息
			//暂时注释掉，允许非法的id
			//Error_Handler();
		}
		if (config.rx_callback == nullptr)
		{
			//等到添加日志模块后，写报错信息
		}

		handle_ = config.handle;
		tx_id_  = config.tx_id;
		rx_id_  = config.rx_id;
		//tx_callback_ = config.tx_callback;
		rx_callback_ = config.rx_callback;

		tx_header_.StdId = tx_id_;
		tx_header_.IDE   = CAN_ID_STD;
		tx_header_.RTR   = CAN_RTR_DATA;
		tx_header_.DLC   = 0x08; //默认长度为8，在发送时可更改

		// 在第一个实例被创建后，统一启动三个fdcan服务
		if (can1_filter_idx_ == 0 && can2_filter_idx_ == 0 && can3_filter_idx_ == 0)
		{
			serviceInit();
		}

		addFilter();

		uint8_t can_dev = handle_->Instance == CAN1 ? 0 : (handle_->Instance == CAN2 ? 1 : 2); // 理论上不会出现2

		// 加入实例列表（尾部插入）
		if (instance_count_[can_dev] < MX_INS_NUM)
		{
			instance_[can_dev][instance_count_[can_dev]++] = this;
		}
	}

	CANInstance::~CANInstance()
	{
		if (!handle_) return;
		uint8_t can_dev = handle_->Instance == CAN1 ? 0 : (handle_->Instance == CAN2 ? 1 : 2);

		for (uint8_t i = 0; i < instance_count_[can_dev]; ++i)
		{
			if (instance_[can_dev][i] == this)
			{
				// 用最后一个覆盖当前项，收缩计数，避免“空洞”
				instance_[can_dev][i]                            = instance_[can_dev][instance_count_[can_dev] - 1];
				instance_[can_dev][instance_count_[can_dev] - 1] = nullptr;
				--instance_count_[can_dev];
				break;
			}
		}
	}

	void CANInstance::addFilter()
	{
		// 如果超出单条总线最大负载就报错
		if (can1_filter_idx_ > MX_INS_NUM ||
			can2_filter_idx_ > MX_INS_NUM ||
			can3_filter_idx_ > MX_INS_NUM)
		{
			Error_Handler();
		}

		CAN_FilterTypeDef filter_cfg;

		// 14的来源见FilterBank的注释
		filter_cfg.FilterBank           = handle_->Instance == CAN1 ? (can1_filter_idx_++) : (14 + can2_filter_idx_++);
		filter_cfg.FilterMode           = CAN_FILTERMODE_IDLIST;
		filter_cfg.FilterScale          = CAN_FILTERSCALE_32BIT;
		filter_cfg.FilterIdHigh         = (rx_id_ << 5);
		filter_cfg.FilterIdLow          = (rx_id_ << 5);
		filter_cfg.FilterMaskIdHigh     = 0x0000;
		filter_cfg.FilterMaskIdLow      = 0x0000;
		filter_cfg.FilterActivation     = ENABLE;
		filter_cfg.FilterFIFOAssignment = CAN_RX_FIFO0;
		filter_cfg.SlaveStartFilterBank = 14;

		if (HAL_CAN_ConfigFilter(handle_, &filter_cfg) != HAL_OK)
		{
			Error_Handler();
		}
	}

	void CANInstance::serviceInit()
	{
		// 中断设置
		uint32_t CAN_ActiveITs = CAN_IT_RX_FIFO0_MSG_PENDING;

		// 初始化两个CAN外设
		HAL_CAN_Start(&hcan1);
		HAL_CAN_ActivateNotification(&hcan1, CAN_ActiveITs);

		HAL_CAN_Start(&hcan2);
		HAL_CAN_ActivateNotification(&hcan2, CAN_ActiveITs);
	}

	void CANInstance::setDataLength(uint8_t length)
	{
		if (length > 8 || length == 0)
		{
			// 长度错误处理
			Error_Handler();
		}
		tx_header_.DLC = length;
	}

	//常规调用方法，发送和接收id依赖实例生成时的配置
	bool CANInstance::send(const msg_t& msg, uint16_t block_timeout_us)
	{
		if (handle_ == nullptr || msg.data == nullptr || msg.length > MX_MSG_LEN)
		{
			return false;
		}

		memcpy(tx_buff_, msg.data, msg.length);
		tx_header_.DLC = msg.length;

		// 4. 检查硬件FIFO状态，如果满则等待
		if (DWTInstance::isInited())
		{
			uint32_t start_time = DWTInstance::getTimeline_us();
			while (HAL_CAN_GetTxMailboxesFreeLevel(handle_) == 0)
			{
				if ((DWTInstance::getTimeline_us() - start_time) > block_timeout_us)
				{
					//todo 如果总线接不到ACK，目前会被持续卡死。需要添加某个机制去清空fifo队列
					return false; // 超时
				}
			}
		}


		// 5. 发送消息
		static uint32_t         tx_mailbox;
		const HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(handle_, &tx_header_, tx_buff_, &tx_mailbox);
		if (status != HAL_OK)
		{
			return false;
		}

		return true;
	}

	//临时发送用的
	bool CANInstance::send(uint32_t target_id,  const msg_t &msg, uint16_t block_timeout_us)
	{
		// 1. 参数检查
		if (handle_ == nullptr || msg.data == nullptr || msg.length > MX_MSG_LEN) {
			return false;
		}

		// 2. 构造临时发送头（基于默认配置）
		CAN_TxHeaderTypeDef temp_header = tx_header_;
		temp_header.StdId               = target_id;
		temp_header.DLC                 = msg.length;

		// 3. 复制数据到发送缓冲区
		uint8_t temp_buff[MX_MSG_LEN];
		if (msg.length > 0)
		{
			memcpy(temp_buff, msg.data, msg.length);
		}

		// 4. 检查 FIFO 状态，如果满则等待
		if (DWTInstance::isInited())
		{
			uint32_t start_time = DWTInstance::getTimeline_us();
			while (HAL_CAN_GetTxMailboxesFreeLevel(handle_) == 0)
			{
				if ((DWTInstance::getTimeline_us() - start_time) > block_timeout_us)
				{
					return false; // 超时
				}
			}
		}

		// 5. 发送消息
		static uint32_t   tx_mailbox;
		HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(handle_, &temp_header, temp_buff, &tx_mailbox);
		if (status != HAL_OK)
		{
			return false;
		}

		return true;
	}


	void CANInstance::RxFifoCallback(CAN_HandleTypeDef* hcan, uint32_t fifo)
	{
		uint8_t can_dev = hcan->Instance == CAN1 ? 0 : (hcan->Instance == CAN2 ? 1 : 2);

		// 处理RX FIFO中的所有新消息
		CAN_RxHeaderTypeDef rx_header;
		uint8_t             rx_data[MX_MSG_LEN];

		while (HAL_CAN_GetRxMessage(hcan, fifo, &rx_header, rx_data) == HAL_OK)
		{
			// 遍历当前 CAN 外设的有效实例
			for (uint8_t i = 0; i < instance_count_[can_dev]; ++i)
			{
				CANInstance* instance = instance_[can_dev][i];
				if (instance && instance->rx_id_ == rx_header.StdId)
				{
					memcpy(instance->rx_buff_, rx_data, rx_header.DLC);
					instance->rx_len_ = rx_header.DLC;
					if (instance->rx_callback_ != nullptr)
					{
						//回调函数要求同时获取id，data，len
						//					instance->rx_callback_(rx_header.Identifier,
						//																 instance->rx_buff_,
						//																 instance->rx_len_);
						//回调函数要求data和len，因为实例化时已经指定了rx_id这里不再加了
						instance->rx_callback_(instance->rx_buff_,
												instance->rx_len_);
					}
					break; // 保持你当前“命中一个就退出”的语义不变
				}
			}
		}
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	ega::CANInstance::RxFifoCallback(hcan, CAN_RX_FIFO0);
}
