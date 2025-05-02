#pragma once
#ifndef APP_MINI_CONTROLLER_HPP
#define APP_MINI_CONTROLLER_HPP

#include "../../User/BSP/Remote/Mini/Mini.hpp"
#include "../../User/BSP/SimpleKey/SimpleKey.hpp"
#include "../IRemoteController.hpp"

namespace Mode
{
using namespace BSP::Remote;

/**
 * @brief Mini遥控器实现
 */
class MiniRemoteController : public IRemoteController
{
  public:
    MiniRemoteController()
    {
        // 初始化APP::Key::SimpleKey类
        key_paused = APP::Key::SimpleKey();
        key_fn_left = APP::Key::SimpleKey();
        key_fn_right = APP::Key::SimpleKey();
    }

    bool isConnected() const override
    {
        // 由于Mini类没有直接的连接状态检测方法，这里使用一个简单的检查
        // 检查gear是否为未知状态，如果不是未知状态，则认为遥控器已连接
        auto &remote = Mini::Instance();
        return remote.gear() != Mini::Gear::UNKNOWN;
    }

    bool isVisionMode() const override
    {
        auto &remote = Mini::Instance();
        return (remote.trigger() == Mini::Switch::UP);
    }

    bool isLaunchMode() const override
    {
        return key_fn_left.getToggleState();
    }

    bool isKeyboardMode() const override
    {
        auto &remote = Mini::Instance();
        return (remote.gear() == Mini::Gear::DOWN);
    }

    bool isStopMode() const override
    {
        // 因为getClick不能在const成员函数中调用，我们需要创建一个非const的副本
        MiniRemoteController *nonConstThis = const_cast<MiniRemoteController *>(this);

        // 短按返回true，长按返回false
        if (key_paused.getLongPress())
        {
            return false; // 长按返回0
        }
        return nonConstThis->key_paused.getClick(); // 短按返回1
    }

    bool isUniversalMode() const override
    {
        auto &remote = Mini::Instance();
        return (remote.gear() == Mini::Gear::UP);
    }

    bool isFollowMode() const override
    {
        auto &remote = Mini::Instance();
        return (remote.gear() == Mini::Gear::MIDDLE);
    }

    bool isRotatingMode() const override
    {
        auto &remote = Mini::Instance();
        return (remote.gear() == Mini::Gear::MIDDLE);
    }

    float getLeftX() const override
    {
        auto &remote = Mini::Instance();
        // Mini遥控器的正确API使用remoteLeft().x获取左摇杆X值
        return remote.remoteLeft().x;
    }

    float getLeftY() const override
    {
        auto &remote = Mini::Instance();
        // Mini遥控器的正确API使用remoteLeft().y获取左摇杆Y值
        return remote.remoteLeft().y;
    }

    float getRightX() const override
    {
        auto &remote = Mini::Instance();
        // Mini遥控器的正确API使用remoteRight().x获取右摇杆X值
        return remote.remoteRight().x;
    }

    float getRightY() const override
    {
        auto &remote = Mini::Instance();
        // Mini遥控器的正确API使用remoteRight().y获取右摇杆Y值
        return remote.remoteRight().y;
    }

    float getMouseVelX() const override
    {
        auto &remote = Mini::Instance();
        return remote.mouseVel().x;
    }

    float getMouseVelY() const override
    {
        auto &remote = Mini::Instance();
        return remote.mouseVel().y;
    }

    float getSw() const override
    {
        auto &remote = Mini::Instance();
        return remote.sw().x;
    }

    void update() override
    {
        auto &remote = Mini::Instance();
        key_paused.update(remote.paused() == Mini::Switch::UP);
        key_fn_left.update(remote.fnLeft() == Mini::Switch::UP);
        key_fn_right.update(remote.fnRight() == Mini::Switch::UP);
    }

  private:
    APP::Key::SimpleKey key_paused;
    APP::Key::SimpleKey key_fn_left;
    APP::Key::SimpleKey key_fn_right;
};

} // namespace Mode

#endif // APP_MINI_CONTROLLER_HPP