#include "RemoteSwitchTask.hpp"

#include "../User/APP/KeyBorad/KeyBroad.hpp"
#include "../User/APP/Mod/RemoteModeManager.hpp"
#include "../User/Task/CommunicationTask.hpp"
#include "../User/Task/GimbalTask.hpp"
#include "../User/Task/ShootTask.hpp"

#include "../User/BSP/Remote/Mini/Mini.hpp"

#include "cmsis_os2.h"

/**
 * @brief 直接放在1000hz任务运行的话会出现误判掉线问题，
 *        因为频率过快导致的，所以需要降低频率
 * @param argument
 */
void keyBoradUpdata();
void BoosterUpState();
void GimbalUpState();

void RemoteSwitchTask(void *argument)
{
    for (;;)
    {
        // 更新遥控器状态
        auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
        remote->update();

        // 更新键盘状态
        APP::Key::KeyBroad::Instance().Update(remote->getKeybroad());

        keyBoradUpdata();
        BoosterUpState();
        GimbalUpState();

        osDelay(5);
    }
}
int8_t shift;
void keyBoradUpdata()
{
    using namespace APP::Key;

    // 底盘方向按键
    auto W = KeyBroad::Instance().getPress(KeyBroad::KEY_W);
    auto A = KeyBroad::Instance().getPress(KeyBroad::KEY_A);
    auto S = KeyBroad::Instance().getPress(KeyBroad::KEY_S);
    auto D = KeyBroad::Instance().getPress(KeyBroad::KEY_D);

    // 小陀螺切换
    auto Q = KeyBroad::Instance().getKeyToggle(KeyBroad::KEY_Q);

    // 按住shift
    auto SHITF = KeyBroad::Instance().getPress(KeyBroad::KEY_SHIFT);
    shift = KeyBroad::Instance().getPress(KeyBroad::KEY_SHIFT);
    // 下降沿加减功率
    auto F = KeyBroad::Instance().getFallingEdge(KeyBroad::KEY_F);
    auto V = KeyBroad::Instance().getFallingEdge(KeyBroad::KEY_V);

    // 点击刷新
    auto CTRL = KeyBroad::Instance().getKeyClick(KeyBroad::KEY_CTRL);

    // 点击切换视觉模式
    auto Vision = KeyBroad::Instance().getKeyClick(KeyBroad::KEY_C);

    // 一键掉头
    auto X = KeyBroad::Instance().getKeyClick(KeyBroad::KEY_X);

    auto C = KeyBroad::Instance().getKeyClick(KeyBroad::KEY_C);

    //   前进后退
    if (W)
        Gimbal_to_Chassis_Data.set_LY(-1);
    else if (S)
        Gimbal_to_Chassis_Data.set_LY(1);
    else
        Gimbal_to_Chassis_Data.set_LY(0);

    // 左右
    if (A)
        Gimbal_to_Chassis_Data.set_LX(1);
    else if (D)
        Gimbal_to_Chassis_Data.set_LX(-1);
    else
        Gimbal_to_Chassis_Data.set_LX(0);

    // 小陀螺
    if (Q)
        Gimbal_to_Chassis_Data.set_Rotating_vel(220);
    else
        Gimbal_to_Chassis_Data.set_Rotating_vel(0);

    // 按住就是shitf不是长按，不需要if
    Gimbal_to_Chassis_Data.set_Shift(SHITF);

    // 增加功率
    if (F)
        Gimbal_to_Chassis_Data.setPower(10);

    // 减功率
    if (V)
        Gimbal_to_Chassis_Data.setPower(-10);

    if (C)
        Gimbal_to_Chassis_Data.setFly(100);

    // 刷新
    if (CTRL)
    {
        Gimbal_to_Chassis_Data.set_UIF5(CTRL);
        Gimbal_to_Chassis_Data.setFly(0);
    }

    if (X)
    {
        TASK::GIMBAL::gimbal.setTrueAround();
    }

    static uint8_t vision_mode = 1;
    if (Vision)
    {
        vision_mode++;
    }
    else if (vision_mode > 3)
    {
        vision_mode = 1;
    }
    Communicat::vision.setVisionMode(vision_mode);
    Gimbal_to_Chassis_Data.setVisionMode(vision_mode);
}

void BoosterUpState()
{
    using namespace APP::Key;

    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    // 点击开启发射机构
    auto G = KeyBroad::Instance().getKeyClick(KeyBroad::KEY_G);
    auto B = KeyBroad::Instance().getKeyClick(KeyBroad::KEY_B);

    static bool booster_enabled = false;

    // 按G键开启，按B键关闭
    if (G || remote->isLaunchMode())
        booster_enabled = true;
    if (B || remote->isStopMode())
        booster_enabled = false;

    // 如果未启用或处于停止模式，则禁用
    if (!booster_enabled || remote->isStopMode())
    {
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::DISABLE);
    }
    else if (remote->isVisionFireMode())
    {
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::ONLY);
    }
    else if (remote->isLaunchMode())
    {
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::AUTO);
    }
    else
    {
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::AUTO);
    }
}
void GimbalUpState()
{
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
    if (remote->isStopMode())
    {
        TASK::GIMBAL::gimbal.setNowStatus(TASK::GIMBAL::DISABLE);
    }
    else if (remote->isVisionMode() && Communicat::vision.getVisionFlag())
    {
        TASK::GIMBAL::gimbal.setNowStatus(TASK::GIMBAL::VISION);
    }
    else if (remote->isKeyboardMode())
    {
        TASK::GIMBAL::gimbal.setNowStatus(TASK::GIMBAL::KEYBOARD);
    }
    else
    {
        TASK::GIMBAL::gimbal.setNowStatus(TASK::GIMBAL::NORMOL);
    }
}
