//
// Created by An on 2025/11/21.
//

#ifndef MODULE_KEY_ON_BOARD_H_
#define MODULE_KEY_ON_BOARD_H_

#include <optional>
#include "driver_gpio.h"

namespace ega
{
    class KeyOnBoard
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        using KeyCallback = std::function<void()>;

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        struct Config
        {
            KeyCallback click_callback = nullptr;
            KeyCallback release_callback = nullptr;
            /// 是否使用防抖
            bool use_debounce = false; //todo
        };

        /* ====================== 3. 静态接口 ====================== */
    public:
        static KeyOnBoard& getInstance();

        static void init(const Config& config = {
            .click_callback = nullptr,
            .release_callback = nullptr,
            .use_debounce = false,
        });

        static GPIO_PinState read();
        static bool isPressed();
        /// 注册按键点击回调函数
        static void setClickCallback(const KeyCallback& callback);
        /// 注册按键释放回调函数
        static void setReleaseCallback(const KeyCallback& callback);

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        KeyOnBoard(const KeyOnBoard&) = delete;
        KeyOnBoard& operator=(const KeyOnBoard&) = delete;

    private:
        KeyOnBoard() = default;
        ~KeyOnBoard() = default;

        /* ====================== 5. 公共接口 ====================== */

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        void KeyInterruptCallback() const;

    private:
        /// 按键引脚实例
        std::optional<GPIOInstance> key_instance_;
        KeyCallback click_callback_ = nullptr;
        KeyCallback release_callback_ = nullptr;

        bool use_debounce_ = false;
    };
}
#endif //MODULE_KEY_ON_BOARD_H_
