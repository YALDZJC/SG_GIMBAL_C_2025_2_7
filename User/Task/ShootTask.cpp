#include "../Task/ShootTask.hpp"
#include "../Task/CommunicationTask.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"

#include "../APP/Tools.hpp"
#include "../App/Mod/RemoteModeManager.hpp"

#include "../BSP/Motor/Dji/DjiMotor.hpp"

#include "cmsis_os2.h"
float hz_send;

TASK::Shoot::Class_ShootFSM shoot_fsm;
void ShootTask(void *argument)
{
    for (;;)
    {
        // hz_send += 0.001;
        // Communicat::Vision_Data.time_demo();
        shoot_fsm.Control();
        osDelay(1);
    }
}

namespace TASK::Shoot
{
// 构造函数定义，使用初始化列表
Class_ShootFSM::Class_ShootFSM()
    : adrc_friction_L_vel(Alg::LADRC::TDquadratic(200, 0.001), 10, 25, 1, 0.001, 16384),
      adrc_friction_R_vel(Alg::LADRC::TDquadratic(200, 0.001), 10, 25, 1, 0.001, 16384),
      adrc_Dail_vel(Alg::LADRC::TDquadratic(200, 0.001), 5, 40, 0.9, 0.001, 16384),
      Heat_Limit(100, 10.0f) // 示例参数：窗口大小50，阈值10.0
{
    // 其他初始化逻辑（如果有）
}

void Class_JammingFSM::UpState()
{
    Status[Now_Status_Serial].Count_Time++;

    auto Motor_Friction = BSP::Motor::Dji::Motor3508;
    auto Motor_Dail = BSP::Motor::Dji::Motor2006;

    switch (Now_Status_Serial)
    {
    case (Jamming_Status::NORMAL): {
        // 正常状态
        if (Motor_Dail.getTorque(1) >= stall_torque)
        {
            // 大扭矩->卡弹嫌疑状态
            Set_Status(Jamming_Status::SUSPECT);
        }

        break;
    }
    case (Jamming_Status::SUSPECT): {
        // 卡弹嫌疑状态

        if (Status[Now_Status_Serial].Count_Time >= stall_torque)
        {
            // 长时间大扭矩->卡弹反应状态
            Set_Status(Jamming_Status::PROCESSING);
        }
        else if (Motor_Dail.getTorque(1) < stall_torque)
        {
            // 短时间大扭矩->正常状态
            Set_Status(Jamming_Status::NORMAL);
        }

        break;
    }
    case (Jamming_Status::PROCESSING): {
        // 卡弹反应状态->准备卡弹处理
        Booster->setTargetDailTorque(3000);

        Set_Status(Jamming_Status::NORMAL);

        break;
    }
    }
}

void Class_ShootFSM::UpState()
{
    switch (Now_Status_Serial)
    {
    case (Booster_Status::DISABLE): {
        // 如何失能状态，拨盘力矩为0，摩擦轮期望值为0
        adrc_friction_L_vel.setTarget(0.0f);
        adrc_friction_R_vel.setTarget(0.0f);

        target_Dail_torque = 0.0f;
        break;
    }
    case (Booster_Status::STOP): {
        // 停止状态，拨盘期望值为0
        //        adrc_friction_L_vel.setTarget(target_friction_omega);
        //        adrc_friction_R_vel.setTarget(-target_friction_omega);

        adrc_Dail_vel.setTarget(0);

        break;
    }
    case (Booster_Status::ONLY): {
        // 单发模式
        break;
    }
    case (Booster_Status::AUTO): {
        // 连发模式
        adrc_friction_L_vel.setTarget(target_friction_omega);
        adrc_friction_R_vel.setTarget(-target_friction_omega);

        auto *remote = Mode::RemoteModeManager::Instance().getActiveController();
        target_dail_omega = remote->getSw() * 400.0f;

        adrc_Dail_vel.setTarget(target_dail_omega);

        break;
    }
    }
}

void Class_ShootFSM::Control(void)
{
    auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

    auto velL = BSP::Motor::Dji::Motor3508.getVelocityRpm(1);
    auto velR = BSP::Motor::Dji::Motor3508.getVelocityRpm(2);
    auto DailVel = BSP::Motor::Dji::Motor2006.getVelocityRpm(1);

    if (remote->isStopMode())
    {
        Now_Status_Serial = Booster_Status::DISABLE;
    }
    else if (remote->isLaunchMode())
    {
        Now_Status_Serial = Booster_Status::AUTO;
    }
    else
    {
        Now_Status_Serial = Booster_Status::STOP;
    }

    UpState();

    // 控制摩擦轮
    adrc_friction_L_vel.UpData(velL);
    adrc_friction_R_vel.UpData(velR);
    adrc_Dail_vel.UpData(DailVel);

    target_Dail_torque = adrc_Dail_vel.getU();

    JammingFMS.UpState();
    HeatLimit();

    // 拨盘发送
    //  Tools.vofaSend(adrc_Dail_vel.getZ1(), adrc_Dail_vel.getTarget(), adrc_Dail_vel.getFeedback(), 0, 0, 0);
    // 摩擦轮发送
    // Tools.vofaSend(adrc_friction_L_vel.getZ1(), adrc_friction_L_vel.getTarget(), adrc_friction_L_vel.getFeedback(),
    //                adrc_friction_R_vel.getZ1(), adrc_friction_R_vel.getTarget(), adrc_friction_R_vel.getFeedback());

    // 火控
    Tools.vofaSend(BSP::Motor::Dji::Motor3508.getTorque(1), BSP::Motor::Dji::Motor3508.getTorque(2),
                   Heat_Limit.getCurSum(), Heat_Limit.getFireNum(), 0, 0);

    CAN_Send();
}

void Class_ShootFSM::HeatLimit()
{
    auto CurL = BSP::Motor::Dji::Motor3508.getTorque(1);
    auto CurR = BSP::Motor::Dji::Motor3508.getTorque(2);

    auto velL = BSP::Motor::Dji::Motor3508.getVelocityRpm(1);
    auto velR = BSP::Motor::Dji::Motor3508.getVelocityRpm(2);

    Heat_Limit.setBoosterHeat(100, 10);
    Heat_Limit.setFrictionCurrent(CurL, CurR);
    Heat_Limit.setFrictionVel(velL, velR);
    // Heat_Limit.setTargetFire();

    Heat_Limit.UpData();
}

void Class_ShootFSM::CAN_Send(void)
{
    auto Motor_Friction = BSP::Motor::Dji::Motor3508;
    auto Motor_Dail = BSP::Motor::Dji::Motor2006;

    BSP::Motor::Dji::Motor3508.setCAN(adrc_friction_L_vel.getU(), 2);
    BSP::Motor::Dji::Motor3508.setCAN(adrc_friction_R_vel.getU(), 3);

    BSP::Motor::Dji::Motor3508.setCAN(target_Dail_torque, 1);

    Motor_Friction.sendCAN(&hcan1, 1);
    //    Motor_Dail.sendCAN(&hcan1, 1);
}
} // namespace TASK::Shoot
