//
// Created by An on 2025/11/21.
//
#include "driver_gpio.h"

namespace
{
    GPIO_TypeDef* getExtiPort(uint16_t pin)
    {
        uint32_t line = 0;
        while (line < 16U && pin != static_cast<uint16_t>(1UL << line))
        {
            ++line;
        }
        if (line >= 16U)
        {
            return nullptr;
        }

        const uint32_t shift = (line & 0x3U) * 4U;
        const uint32_t port_code =
            (SYSCFG->EXTICR[line >> 2U] >> shift) & 0xFU;
        switch (port_code)
        {
        case 0U: return GPIOA;
        case 1U: return GPIOB;
        case 2U: return GPIOC;
        case 3U: return GPIOD;
        case 4U: return GPIOE;
        case 5U: return GPIOF;
        case 6U: return GPIOG;
        case 7U: return GPIOH;
        case 8U: return GPIOI;
        default: return nullptr;
        }
    }
}

namespace ega
{
    /// 静态成员初始化
    std::array<GPIOInstance*, GPIOInstance::MX_INS_NUM> GPIOInstance::instance_list_{};
    uint8_t GPIOInstance::instance_count_ = 0;

    GPIOInstance::GPIOInstance(const GPIOInstance::Config& config):
        port_(config.port),
        pin_(config.pin),
        default_state_(config.default_state),
        exti_mode_(config.exti_mode),
        exti_callback_(config.exti_callback)
    {
        // 加入实例列表
        if (instance_count_ < MX_INS_NUM)
        {
            instance_list_[instance_count_++] = this;
        }

    }

    GPIOInstance::~GPIOInstance()
    {
        for (uint8_t i = 0; i < instance_count_; i++)
        {
            if (instance_list_[i] == this)
            {
                // 用最后一个覆盖当前元素
                instance_list_[i] = instance_list_[instance_count_ - 1];
                instance_list_[instance_count_ - 1] = nullptr;
                --instance_count_;
                break;
            }
        }
    }


}

//重载HAL库函数
extern "C" {
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    using namespace ega;
    GPIO_TypeDef* const exti_port = getExtiPort(GPIO_Pin);
    // 如有必要,可以根据default_state_和HAL_GPIO_ReadPin来判断是上升沿还是下降沿/rise&fall等
    // 本来用的就比较少，暂时不在GPIO驱动层级内实现。板载按钮在自己的回调里面实现
    for (auto ins : GPIOInstance::instance_list_)
    {
        if (ins != nullptr &&
            ins->port_ == exti_port &&
            ins->pin_ == GPIO_Pin &&
            ins->exti_callback_ != nullptr)
        {
            ins->exti_callback_();
            return;
        }
    }
}
}
