#pragma once
#ifndef APP_REMOTE_MODE_MANAGER_HPP
#define APP_REMOTE_MODE_MANAGER_HPP

#include "../User/Task/CommunicationTask.hpp"
#include "DR16Controller.hpp"
#include "IRemoteController.hpp"
#include "MiniController.hpp"

namespace Mode
{

/**
 * @brief 遥控器模式管理器，统一接口并处理遥控器断联情况
 */
class RemoteModeManager
{
  public:
    // 单例模式
    static RemoteModeManager &Instance()
    {
        static RemoteModeManager instance;
        return instance;
    }

    // 初始化管理器
    void init()
    {
        m_dr16Controller = new DR16RemoteController();
        m_miniController = new MiniRemoteController();
    }

    // 清理资源
    ~RemoteModeManager()
    {
        delete m_dr16Controller;
        delete m_miniController;
    }

    // 更新遥控器状态
    void update()
    {
        m_dr16Controller->update();
        m_miniController->update();

        // 获取当前应该使用的遥控器
        determineActiveController();
    }

    // 获取当前活动的遥控器
    IRemoteController *getActiveController()
    {
        return m_activeController;
    }

    // 是否有可用的遥控器
    bool hasActiveController() const
    {
        return m_activeController != nullptr;
    }

  private:
    // 决定哪个遥控器是当前活动的
    void determineActiveController()
    {
        if (m_dr16Controller->isConnected())
        {
            m_activeController = m_dr16Controller;
        }
        else if (m_miniController->isConnected())
        {
            m_activeController = m_miniController;
        }
        else
        {
            m_activeController = nullptr;
        }
    }

    // 私有构造函数，防止外部创建实例
    RemoteModeManager() : m_dr16Controller(nullptr), m_miniController(nullptr), m_activeController(nullptr)
    {
    }

    RemoteModeManager(const RemoteModeManager &) = delete;
    RemoteModeManager &operator=(const RemoteModeManager &) = delete;

    DR16RemoteController *m_dr16Controller;
    MiniRemoteController *m_miniController;
    IRemoteController *m_activeController;
};

// 提供全局函数接口，与原来的接口保持一致
namespace Gimbal
{
inline bool Vision()
{
    auto &manager = RemoteModeManager::Instance();
    if (!manager.hasActiveController())
        return false;
    return manager.getActiveController()->isVisionMode();
}

inline bool Launch()
{
    auto &manager = RemoteModeManager::Instance();
    if (!manager.hasActiveController())
        return false;
    return manager.getActiveController()->isLaunchMode();
}

inline bool KeyBoard()
{
    auto &manager = RemoteModeManager::Instance();
    if (!manager.hasActiveController())
        return false;
    return manager.getActiveController()->isKeyboardMode();
}

inline bool Stop()
{
    auto &manager = RemoteModeManager::Instance();
    if (!manager.hasActiveController())
        return true; // 没有遥控器时默认停止
    return manager.getActiveController()->isStopMode();
}
} // namespace Gimbal

namespace Chassis
{
inline bool Universal()
{
    auto &manager = RemoteModeManager::Instance();
    if (!manager.hasActiveController())
        return false;
    return manager.getActiveController()->isUniversalMode();
}

inline bool Follow()
{
    auto &manager = RemoteModeManager::Instance();
    if (!manager.hasActiveController())
        return false;
    return manager.getActiveController()->isFollowMode();
}

inline bool Rotating()
{
    auto &manager = RemoteModeManager::Instance();
    if (!manager.hasActiveController())
        return false;
    return manager.getActiveController()->isRotatingMode();
}

inline bool KeyBoard()
{
    auto &manager = RemoteModeManager::Instance();
    if (!manager.hasActiveController())
        return false;
    return manager.getActiveController()->isKeyboardMode();
}

inline bool Stop()
{
    auto &manager = RemoteModeManager::Instance();
    if (!manager.hasActiveController())
        return true; // 没有遥控器时默认停止
    return manager.getActiveController()->isStopMode();
}

inline void SendRemote()
{
    auto &manager = RemoteModeManager::Instance();
    if (!manager.hasActiveController())
        return;

    auto *controller = manager.getActiveController();
    if (Mode::Gimbal::KeyBoard() == false)
    {
        Gimbal_to_Chassis_Data.set_LX(controller->getLeftX());
        Gimbal_to_Chassis_Data.set_LY(controller->getLeftY());
    }
}
} // namespace Chassis

} // namespace Mode

#endif // APP_REMOTE_MODE_MANAGER_HPP
