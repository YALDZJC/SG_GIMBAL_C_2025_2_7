#include "RemoteSwitchTask.hpp"

#include "../User/APP/KeyBorad/KeyBroad.hpp"
#include "../User/APP/Mod/RemoteModeManager.hpp"
#include "../User/Task/CommunicationTask.hpp"
#include "../User/Task/ShootTask.hpp"
#include "cmsis_os2.h"

/**
 * @brief 直接放在1000hz任务运行的话会出现误判掉线问题，
 *        因为频率过快导致的，所以需要降低频率
 * @param argument
 */
void keyBoradUpdata();
void BoosterUpState();
void RemoteSwitchTask(void *argument)
{
    for (;;)
    {
        // 更新遥控器状态
        auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
        remote->update();

        // 更新按键状态
        APP::Key::KeyBroad::Instance().Update(remote->getKeybroad());
        keyBoradUpdata();
        BoosterUpState();

        osDelay(14); // 遥控器更新频率为70hz
    }
}

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

    // 下降沿加减功率
    auto F = KeyBroad::Instance().getFallingEdge(KeyBroad::KEY_F);
    auto V = KeyBroad::Instance().getFallingEdge(KeyBroad::KEY_V);

    // 点击刷新
    auto CTRL = KeyBroad::Instance().getKeyClick(KeyBroad::KEY_CTRL);

    // 点击开启发射机构
    auto G = KeyBroad::Instance().getKeyClick(KeyBroad::KEY_G);
    auto B = KeyBroad::Instance().getKeyClick(KeyBroad::KEY_B);

    //  前进后退
    if (W)
        Gimbal_to_Chassis_Data.set_LY(1);
    else if (S)
        Gimbal_to_Chassis_Data.set_LY(-1);
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

    // 冲刺
    if (SHITF)
        Gimbal_to_Chassis_Data.set_Shift(SHITF);

    // 增加功率
    if (F)
        Gimbal_to_Chassis_Data.setPower(10);

    // 减功率
    if (V)
        Gimbal_to_Chassis_Data.setPower(-10);

    // 刷新
    if (CTRL)
        Gimbal_to_Chassis_Data.set_UIF5(CTRL);

    // 云台逻辑
    if (G)
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::AUTO);

    if (B)
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::DISABLE);
}

void BoosterState()
{
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    if (remote->isLaunchMode())
    {
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::AUTO);
    }
    else if (remote->isVisionFireMode())
    {
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::ONLY);
    }
    else
    {
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::STOP);
    }

    if (remote->isStopMode())
    {
        TASK::Shoot::shoot_fsm.setNowStatus(TASK::Shoot::Booster_Status::STOP);
    }
}

void GimbalUpState()
{
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    // if(remote->is)
}
