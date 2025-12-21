//
// Created by zhangzhiwen on 25-12-21.
//

#ifndef DRIVER_DWT_H
#define DRIVER_DWT_H

#include "main.h"

namespace ega
{
    class DWTInstance
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */

        /* ====================== 2. 内部类型定义 ====================== */
    public:
        /* 内建时间类,维护时间轴供DWTGetTimeline()使用*/
        struct time_t
        {
            uint32_t s;
            uint16_t ms;
            uint16_t us;
        };
        /* ====================== 3. 静态接口 ====================== */
    public:
        static DWTInstance& getInstance()
        {
            static DWTInstance instance;
            return instance;
        }

        /**
         * @brief 初始化DWT,传入参数为CPU频率,单位MHz
         *
         * @param CPU_Freq_mHz c板为168MHz,A板为180MHz,喵板为480MHz,和时钟树&时钟配置有关
         */
        static void init(uint32_t CPU_Freq_mHz = 168);
        static bool isInited() { return getInstance().inited_; }

        // 当前时间轴,以初始化后的时间为基准
        static float getTimeline_s(void);
        static float getTimeline_ms(void);
        static uint64_t getTimeline_us(void);

        /**
         * @brief 获取两次调用之间的时间间隔,单位为秒/s
         *
         * @param cnt_last 上一次调用的时间戳
         * @return float 时间间隔,单位为秒/s
         */
        static float getDeltaT();

        /* DWTGetDeltaT()的双精度浮点版本 */
        static double getDeltaT64();

        /**
         * @brief DWT延时函数,单位为秒/s
         * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
         * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
         *
         * @param Delay 延时时间,单位为秒/s
         */
        static void delay(float Delay);

        /* ====================== 4. 构造 / 析构 ====================== */
    private:
        // 单例相关
        DWTInstance() = default;
        DWTInstance(const DWTInstance&) = delete;
        DWTInstance& operator=(const DWTInstance&) = delete;
        ~DWTInstance() = default;

        /* ====================== 5. 公共接口 ====================== */

        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        // 初始化时获取
        uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;

        uint32_t cyc_round_cnt_; // CYCCNT溢出次数,用于计算时间轴
        uint64_t CYCCNT64; // CYCCNT的总计数(判断溢出后会直接增加uint32_t_MAX)
        uint32_t cnt_; // 每个instance的计数器,用于提供间隔时间

        bool inited_ = false; //标记其是否已经被初始化

        time_t time{};

    private:
        /**
         * @brief 私有函数,用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_RountCount
         * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
         *
         * @todo 更好的方案是为dwt的时间更新单独设置一个任务?
         *       不过,使用dwt的初衷是定时不被中断/任务等因素影响,因此该实现仍然有其存在的意义
         *
         */
        void CNT_Update(void);

        /**
         * @brief DWT更新时间轴函数,会被三个timeline函数调用
         * @attention 如果长时间不调用timeline函数,则需要手动调用该函数更新时间轴
         *            否则CYCCNT溢出后定时和时间轴不准确
         */
        void SysTimeUpdate(void);

    };
}


#endif //DRIVER_DWT_H
