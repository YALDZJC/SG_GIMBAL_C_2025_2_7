#pragma once
#ifndef APP_DR16_CONTROLLER_HPP
#define APP_DR16_CONTROLLER_HPP

#include "../User/BSP/Remote/Dbus/Dbus.hpp"
#include "../User/Task/EvenTask.hpp"
#include "IRemoteController.hpp"

namespace Mode
{
using namespace BSP::Remote;

/**
 * @brief DR16遥控器实现
 */
class DR16RemoteController : public IRemoteController
{
  public:
    bool isConnected() const override
    {
        return !Dir_Event.getDir_Remote(); // Dir_Remote为true表示断连
    }

    bool isVisionMode() const override
    {
        return (dr16.switchLeft() != Dr16::Switch::MIDDLE) && (dr16.switchRight() == Dr16::Switch::MIDDLE);
    }

    bool isLaunchMode() const override
    {
        return (dr16.switchRight() == Dr16::Switch::UP);
    }

    bool isKeyboardMode() const override
    {
        return (dr16.switchLeft() == Dr16::Switch::MIDDLE) && (dr16.switchRight() == Dr16::Switch::MIDDLE);
    }

    bool isStopMode() const override
    {
        return (dr16.switchLeft() == Dr16::Switch::DOWN) && (dr16.switchRight() == Dr16::Switch::DOWN) ||
               (Dir_Event.getDir_Remote() == true);
    }

    bool isUniversalMode() const override
    {
        return (dr16.switchLeft() == Dr16::Switch::UP);
    }

    bool isFollowMode() const override
    {
        return (dr16.switchLeft() == Dr16::Switch::MIDDLE) && (dr16.switchRight() != Dr16::Switch::MIDDLE);
    }

    bool isRotatingMode() const override
    {
        return (dr16.switchLeft() == Dr16::Switch::DOWN) && (dr16.switchRight() != Dr16::Switch::DOWN);
    }

    float getLeftX() const override
    {
        return dr16.remoteLeft().x;
    }

    float getLeftY() const override
    {
        return dr16.remoteLeft().y;
    }

    void update() override
    {
        // DR16没有需要额外更新的内容
    }
};

} // namespace Mode

#endif // APP_DR16_CONTROLLER_HPP