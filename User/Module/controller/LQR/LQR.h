#pragma once

#include "controller.h"
#include "matrix.h"

namespace ega
{
    /**
     * @brief LQR 控制器
     * @tparam StateDim 状态维度 (x的维度)
     * @tparam OutputDim 输出维度 (u的维度)
     */
    template <std::size_t StateDim, std::size_t OutputDim>
    class LQR:public Controller
    {
        /* ====================== 1. 编译期常量 & 类型别名 ====================== */
    public:
        using State = Matrixf<StateDim,1>;
        using Output = Matrixf<OutputDim,1>;
        using GainMatrix = Matrixf<OutputDim,StateDim>;
        /* ====================== 2. 内部类型定义 ====================== */
    public:
        struct Config
        {
            GainMatrix K;
            Output u_min;//输出下限
            Output u_max;//输出上限
        };

        /* ====================== 3. 静态接口 ====================== */

        /* ====================== 4. 构造 / 析构 ====================== */
    public:
        explicit LQR(const Config& config);

        /* ====================== 5. 实现接口 ====================== */
    public:
        Output calculate(const State& target,const State& measure);
        void clear();
        /* ====================== 6. 受保护接口 ====================== */

        /* ====================== 7. 成员变量 ====================== */
    private:
        GainMatrix K_;
        Output u_min_;//输出下限
        Output u_max_;//输出上限

        State  error_;   // x - x_ref
        Output output_;  // cached output

    };
}


namespace ega
{
    template <std::size_t StateDim, std::size_t ControlDim>
    LQR<StateDim, ControlDim>::LQR(const Config& config):
    K_(config.K),
    u_min_(config.u_min),
    u_max_(config.u_max)
    {

    }

    template <std::size_t StateDim, std::size_t OutputDim>
    typename LQR<StateDim, OutputDim>::Output LQR<StateDim, OutputDim>::calculate(const State& target, const State& measure)
    {
        error_ = measure - target;
        output_ = - K_ * error_;
        //todo output限幅
        return output_;
    }



    template <std::size_t StateDim, std::size_t OutputDim>
    void LQR<StateDim, OutputDim>::clear()
    {
        error_.clear();
        output_.clear();
    }
}
