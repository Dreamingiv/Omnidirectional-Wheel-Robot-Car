//
// Created by An on 2025/11/21.
//

#ifndef ROBOT_MANAGER_H_
#define ROBOT_MANAGER_H_
#include <cstdint>
#include <optional>
#include <variant>
#include "remote_DT7.h"
#include "remote_VT13.h"
#include "remote_FS_I6X.h"
#include "can_comm.h"

/**
 *	@brief 机器人管理器，主要用于保护模式判断
 *	@note
 *	 - 全局单例模式
 *	 - 安全模式会影响电机的通断，详情见motor_task
 *	 - 需要在配置里决定使用的遥控器类型，同步初始化遥控器（因为遥控器是安全模式判断的重要来源）
 *	 - 除了遥控器是否在线外，如果后续有其他因素（电机是否在线等），也需要在update中加入对应的判断
 */
namespace ega
{
    class RobotManager
    {
        /* ====================== 类型定义 ====================== */
    public:
        enum class WorkState
        {
            Protect = 0, //保护状态
            Work, //正常工作
            //其他状态
        };

        enum class BoardType : uint8_t
        {
            OneBoard = 0,
            GimbalBoard,
            ChassisBoard
        };

        enum class RemoteType : uint8_t
        {
            None = 0,
            VT13,
            DT7,
            FS_I6X,
        };
        using RemoteVariant = std::variant<VT13, DT7, FS_I6X>;

        /* ====================== 初始化配置 ====================== */
        struct Config
        {
            //后续如果配置不多，可以考虑把这两项作为模板参数，利用if constexpr实现编译期裁剪
            BoardType board_type = BoardType::OneBoard;
            RemoteType remote_type = RemoteType::FS_I6X;
        };

        /* ====================== 控制指令相关 ====================== */
        //由遥控器解析出机器人控制指令
        struct Command
        {
            struct GimbalCommand
            {

            };
            struct ShootCommand
            {

            };
            struct ChassisCommand
            {
                float vx = 0.0f;
                float vy = 0.0f;
                float wz = 0.0f;
                bool enable = false;
            };

            GimbalCommand gimbal{};
            ShootCommand shoot{};
            ChassisCommand chassis{};
            //其它指令另外再添加
        };

        //todo 是否要让robot_manager管理can_comm板间通信的信息？
        /**
         * 设想：利用can_comm板间信息交互
         * 根据板子类型，决定谁是发送方，谁是接收方。
         * 例如，作为云台板时，GimbalBoardInfo从板子本身获取，ChassisBoardInfo来自can_comm
         * 作为底盘板时，则反过来，ChassisBoardInfo从板子本身获取，GimbalBoardInfo来自can_comm
         * 从而完成云台板和底盘板的信息交互
         */
        // struct GimbalBoardInfo  ///云台板获得的信息，例如yaw轴电机角度等，用于发送给底盘板
        // {
        //
        // };
        // struct ChassisBoardInfo ///底盘板获得的信息，例如裁判系统信息等，用于发送给云台板
        // {
        //
        // };

        /* ====================== 静态接口 ====================== */
    public:
        // ======== 外部静态访问接口 ========
        static RobotManager& getInstance()
        {
            static RobotManager instance; // C++11 线程安全
            return instance;
        }

        //初始化，必须先调用init确定遥控器配置
        static bool init(const Config& config);

        // ======== 更新接口 ========
        // 统一的调度接口，放在robot_task中跑
        static void update();

        // ======== getters ========
        static bool isInited() { return getInstance().inited_; }
        static bool hasRemote() { return getInstance().remote_.has_value(); }
        static WorkState getWorkState() { return getInstance().work_state_; }
        static Command getCommand() { return getInstance().command_; }
        static RemoteVariant& getRemote()
        {
            auto& ins = getInstance();
            return ins.remote_.value();
        }
        static const RemoteVariant& getRemoteConst()
        {
            const auto& ins = getInstance();
            return ins.remote_.value();
        }

        /* ====================== 构造 / 析构 ====================== */
    private:
        // ======== 构造/析构私有化 ========
        RobotManager() = default;

        ~RobotManager() = default;

    public:
        // ======== 禁止拷贝与赋值 ========
        RobotManager(const RobotManager&) = delete;

        RobotManager& operator=(const RobotManager&) = delete;

        /* ====================== 受保护接口 ====================== */
    private:
        static bool isRemoteOnline(const RemoteVariant& r);

        static void updateRemoteLogic(RemoteVariant& r);
        static void updateRemote_DT7(DT7& dt7);
        static void updateRemote_VT13(VT13& vt13);
        static void updateRemote_FS_I6X(FS_I6X& fs_i6x);

        /* ====================== 成员变量 ====================== */
    private:
        bool inited_ = false;

        WorkState work_state_ = WorkState::Protect; //机器人整车状态
        BoardType board_type_;
        RemoteType remote_type_;

        Command command_{};

        std::optional<RemoteVariant> remote_ = std::nullopt; // 只会启用一种遥控器
    };
}
#endif //ROBOT_MANAGER_H_
