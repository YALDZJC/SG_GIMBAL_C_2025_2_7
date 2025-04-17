#include "../Task/ShootTask.hpp"
#include "../Task/CommunicationTask.hpp"

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
    switch(Now_Status_Serial)
    {
        case (Booster_Status::DISABLE): {
            //如何失能状态，拨盘力矩为0，摩擦轮期望值为0
            adrc_friction_L_vel.setTarget(0.0f);
            adrc_friction_R_vel.setTarget(0.0f);

            target_Dail_torque = 0.0f;
            break;
        }
        case (Booster_Status::STOP): {
            // 停止状态，拨盘与摩擦轮期望值都为0
            adrc_friction_L_vel.setTarget(target_friction_omega);
            adrc_friction_R_vel.setTarget(-target_friction_omega);

            adrc_Dail_vel.setTarget(0.0f);
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
            adrc_Dail_vel.setTarget(target_dail_omega);

            break;
        }
    }
}

void Class_ShootFSM::Control(void)
{
    auto velL = BSP::Motor::Dji::Motor3508.getAddAngleRad(1);
    auto velR = BSP::Motor::Dji::Motor3508.getAddAngleRad(2);
    auto DailVel = BSP::Motor::Dji::Motor2006.getAddAngleRad(1);

    // 控制摩擦轮
    adrc_friction_L_vel.UpData(velL);
    adrc_friction_R_vel.UpData(velR);
    adrc_Dail_vel.UpData(DailVel);

    CAN_Set();
    CAN_Send();
}

void Class_ShootFSM::CAN_Set(void)
{
    auto Motor_Friction = BSP::Motor::Dji::Motor3508;
    auto Motor_Dail = BSP::Motor::Dji::Motor2006;

    Motor_Friction.setCAN(adrc_friction_L_vel.getU(), 1);
    Motor_Friction.setCAN(adrc_friction_R_vel.getU(), 2);

    Motor_Dail.setCAN(adrc_Dail_vel.getU(), 1);
}

void Class_ShootFSM::CAN_Send(void)
{
    auto Motor_Friction = BSP::Motor::Dji::Motor3508;
    auto Motor_Dail = BSP::Motor::Dji::Motor2006;

    Motor_Friction.sendCAN(&hcan1, 1);
    Motor_Dail.sendCAN(&hcan1, 1);
}
} // namespace TASK::Shoot
