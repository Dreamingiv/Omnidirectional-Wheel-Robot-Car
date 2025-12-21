//
// Created by An on 2025/11/21.
//
#include "key_on_board.h"

namespace ega
{
    KeyOnBoard& KeyOnBoard::getInstance()
    {
        static KeyOnBoard instance;
        return instance;
    }

    void KeyOnBoard::init(const Config& config)
    {
        //如果已经初始化，直接返回
        if (getInstance().key_instance_.has_value())
        {
            return;
        }
        //生成配置单
        GPIOInstance::Config gpio_config{
            .port = GPIOA, //USER_KEY_GPIO_Port,
            .pin = GPIO_PIN_15, //USER_KEY_Pin,
            .exti_callback = []()
            {
                //软件判断按下/松开
                KeyOnBoard::getInstance().KeyInterruptCallback();
            },

            .default_state = GPIO_PIN_SET, //板载的按键默认是高电平
            .exti_mode = GPIOInstance::EXTI_TRIG_MODE::RISING_FALLING, //上升下降均触发
        };
        //创建按键引脚实例
        getInstance().click_callback_ = config.click_callback;
        getInstance().release_callback_ = config.release_callback;
        getInstance().key_instance_.emplace(gpio_config);
        getInstance().use_debounce_ = config.use_debounce;
    }

    GPIO_PinState KeyOnBoard::read()
    {
        if (!getInstance().key_instance_.has_value())
        {
            return GPIO_PIN_SET;
        }
        return getInstance().key_instance_->read();
    }

    bool KeyOnBoard::isPressed()
    {
        if (!getInstance().key_instance_.has_value())
        {
            return false;
        }
        auto& ins = getInstance().key_instance_.value();
        return ins.read() != ins.getDefaultState();
    }

    void KeyOnBoard::KeyInterruptCallback() const
    {
        if (!key_instance_.has_value())
        {
            return;
        }
        auto& ins = key_instance_.value();
        //判断是否按下
        if (ins.read() != ins.getDefaultState())
        {
            //触发中断，且电平与默认电平不同，判断为按下
            if (click_callback_ != nullptr)
            {
                click_callback_();
            }
        }
        else
        {
            //触发中断，且电平与默认电平相同，判断为抬起
            if (release_callback_ != nullptr)
            {
                release_callback_();
            }
        }
    }

    void KeyOnBoard::setClickCallback(const KeyOnBoard::KeyCallback& callback)
    {
        getInstance().click_callback_ = callback;
    }

    void KeyOnBoard::setReleaseCallback(const KeyOnBoard::KeyCallback& callback)
    {
        getInstance().release_callback_ = callback;
    }
}
