#include "../Task/ShootTask.hpp"
#include "../Task/CommunicationTask.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"

#include "../APP/Tools.hpp"

#include "../BSP/Motor/Dji/DjiMotor.hpp"

#include "cmsis_os2.h"
float hz_send;

void ShootTask(void *argument)
{
    for (;;)
    {
        hz_send += 0.001;
        Communicat::vision.time_demo();
				BSP::Motor::Dji::Motor6020.ISDir();

        TASK::Shoot::shoot_fsm.Control();
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
      Heat_Limit(100, 65.0f) // 示例参数：窗口大小50，阈值10.0
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
        adrc_Dail_vel.setTarget(0.0f);
        break;
    }
    case (Booster_Status::ONLY): {
        // 单发模式
        // 设置摩擦轮速度，与连发模式相同
        adrc_friction_L_vel.setTarget(target_friction_omega);
        adrc_friction_R_vel.setTarget(-target_friction_omega);

        // 检测触发条件(使用开火标志位)
        static bool fired = false;

        key_fire.update(Heat_Limit.getFireNum());

        if (fire_flag == 1)
        {
            if (!fired)
            {
                // 未发射过，设置单发速度
                target_dail_omega = Max_dail_angle;
                target_dail_omega = rpm_to_hz(target_dail_omega);

                // 热量限制
                HeatLimit();
                target_dail_omega = Heat_Limit.getNowFire();

                adrc_Dail_vel.setTarget(-target_dail_omega); // 方向相反
                fired = true;
            }
        }
        else
        {
            // 重置发射状态
            fired = false;
        }

        // 检测到子弹发射后停止
        if (fired && key_fire.getRisingEdge())
        {
            adrc_Dail_vel.setTarget(0.0f);
            // 自动复位开火标志位
            fire_flag = 0;
        }

        break;
    }
    case (Booster_Status::AUTO): {
        // 连发模式
        adrc_friction_L_vel.setTarget(target_friction_omega);
        adrc_friction_R_vel.setTarget(-target_friction_omega);

        auto *remote = Mode::RemoteModeManager::Instance().getActiveController();

        target_dail_omega = remote->getSw() * Max_dail_angle; // 单位为20hz发弹频率

        if (remote->isKeyboardMode())
        {
            target_dail_omega = remote->getMouseKeyLeft() * Max_dail_angle;
        }

        target_dail_omega = rpm_to_hz(target_dail_omega); // 转换单位这里的单位是转轴转一圈的rpm
        HeatLimit();
        target_dail_omega = Heat_Limit.getNowFire();
        adrc_Dail_vel.setTarget(-target_dail_omega); // 方向相反

        break;
    }
    }
}

void Class_ShootFSM::Control(void)
{
    auto velL = BSP::Motor::Dji::Motor3508.getVelocityRpm(1);
    auto velR = BSP::Motor::Dji::Motor3508.getVelocityRpm(2);
    auto DailVel = BSP::Motor::Dji::Motor2006.getVelocityRpm(1);

    UpState();
    // 控制摩擦轮
    adrc_friction_L_vel.UpData(velL);
    adrc_friction_R_vel.UpData(velR);
    adrc_Dail_vel.UpData(DailVel);

    target_Dail_torque = adrc_Dail_vel.getU();

    JammingFMS.UpState();

    // 拨盘发送
    //  Tools.vofaSend(adrc_Dail_vel.getZ1(), adrc_Dail_vel.getTarget(), adrc_Dail_vel.getFeedback(), 0, 0, 0);
    // 摩擦轮发送
    // Tools.vofaSend(adrc_friction_L_vel.getZ1(), adrc_friction_L_vel.getTarget(), adrc_friction_L_vel.getFeedback(),
    //                adrc_friction_R_vel.getZ1(), adrc_friction_R_vel.getTarget(), adrc_friction_R_vel.getFeedback());

    // 火控
//    Tools.vofaSend(Heat_Limit.getFireNum(), Heat_Limit.getNowHeat(), Heat_Limit.getMaxHeat(), 0, 0, 0);

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
    Heat_Limit.setTargetFire(target_dail_omega);

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

float Class_ShootFSM::rpm_to_hz(float tar_hz)
{
    const int slots_per_rotation = 9;       // 拨盘每转一圈的槽位数
    const double seconds_per_minute = 60.0; // 每分钟的秒数

    // 计算每秒需要的转数
    double rotations_per_second = tar_hz / slots_per_rotation;

    // 转换为每分钟转数（RPM）
    double rpm = rotations_per_second * seconds_per_minute;

    return rpm;
}

} // namespace TASK::Shoot
