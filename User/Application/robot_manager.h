//
// Created by An on 2025/11/21.
//

#ifndef ROBOT_MANAGER_H_
#define ROBOT_MANAGER_H_
#include <cstdint>


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
	class RobotManager {
		/* ====================== 1. 编译期常量 & 类型别名 ====================== */

		/* ====================== 2. 内部类型定义 ====================== */
	public:
		enum class WorkState {
			Protect = 0,  //保护状态
			Work,      		//正常工作
			//其他状态
		};

	    enum class RemoteType : uint8_t
	    {
	        NONE,
            VT13,
            DT7,
            FS_I6X,
        };

	public:
		struct Config {
			RemoteType remote_type = RemoteType::NONE;
		};

		/* ====================== 3. 静态接口 ====================== */
	public:
		// ======== 外部静态访问接口 ========
		static RobotManager &getInstance() {
			static RobotManager instance;   // C++11 线程安全
			return instance;
		}

	    //初始化，必须先调用init确定遥控器配置
	    static bool init(const Config &config);

	    static void update();

	    // ======== getters ========
	    static bool isInited() { return getInstance().inited_; }
	    static WorkState getWorkState() { return getInstance().work_state_; }

		/* ====================== 4. 构造 / 析构 ====================== */
	private:
		// ======== 构造/析构私有化 ========
		RobotManager() = default;
		~RobotManager() = default;
	public:
		// ======== 禁止拷贝与赋值 ========
		RobotManager(const RobotManager &) = delete;
		RobotManager &operator=(const RobotManager &) = delete;

		/* ====================== 5. 公共接口 ====================== */

		/* ====================== 6. 受保护接口 ====================== */

		/* ====================== 7. 成员变量 ====================== */
	private:
		bool inited_ = false;
		WorkState work_state_ = WorkState::Protect;//机器人整车状态
	    RemoteType remote_type_ = RemoteType::NONE;
	};
}
#endif //ROBOT_MANAGER_H_
