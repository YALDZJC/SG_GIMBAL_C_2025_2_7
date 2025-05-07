#include "../Task/ShootTask.hpp"
#include "../Task/CommunicationTask.hpp"

#include "../APP/Mod/RemoteModeManager.hpp"

#include "../APP/Tools.hpp"

#include "../BSP/Motor/Dji/DjiMotor.hpp"

#include "cmsis_os2.h"
float hz_send;

uint8_t firetime;
uint8_t firenum;
uint8_t fireHz;

void ShootTask(void *argument)
{
    for (;;)
    {
        hz_send += 0.001;
        Communicat::vision.time_demo();

        // 设置视觉开火位
        TASK::Shoot::shoot_fsm.setFireFlag(Communicat::vision.get_fire_num());
        //        firetime++;
        //        if (firetime > fireHz)
        //        {
        //            firenum = 1;
        //            firetime = 0;
        //        }
        //        else
        //        {
        //            firenum = 0;
        //        }
        //        TASK::Shoot::shoot_fsm.setFireFlag(firenum);

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
      // 位置pid增益
      Kpid_Dail_pos(2, 0, 0),
      // 速度pid增益
      Kpid_Dail_vel(60, 0, 0),
      // 热量限制初始化
      Heat_Limit(100, 50.0f) // 示例参数：窗口大小50，阈值10.0
{
    // 初始化卡弹检测状态机
    JammingFMS.Set_Status(Jamming_Status::NORMAL);
    // 将当前射击状态机实例传递给卡弹检测状态机
    JammingFMS.setBooster(this);
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

        if (Status[Now_Status_Serial].Count_Time >= stall_time)
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

        if (Status[Now_Status_Serial].Count_Time > stall_stop)
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

        // 热量限制（滑动窗口，需要持续计算）
        HeatLimit();

        // 获取当前角度
        float current_angle = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);

        static bool fired = false;
        static bool allow_fire = true; // 添加热量限制允许发射标志

        // 热量限制检查
        // getNowFire()返回限制后的发射频率（Hz），如果为0表示禁止发射
        allow_fire = Heat_Limit.getNowFire() > 0.0f;

        if (fire_flag == 1 && allow_fire)
        {
            if (!fired)
            {
                // 设置目标位置
                Dail_target_pos = current_angle - 40.0f;
                fired = true;
            }
            else
            {
                // 自动复位开火标志位
                fire_flag = 0;
            }
        }
        else
        {
            // 重置发射状态
            fired = false;
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

        HeatLimit();
        target_dail_omega = Tools.clamp(target_dail_omega, Heat_Limit.getNowFire(), 0);
        target_dail_omega = rpm_to_hz(target_dail_omega); // 转换单位这里的单位是转轴转一圈的rpm
        pid_Dail_vel.setTarget(-target_dail_omega);
        break;
    }
    }
}

void Class_ShootFSM::Control(void)
{
    auto velL = BSP::Motor::Dji::Motor3508.getVelocityRpm(1);
    auto velR = BSP::Motor::Dji::Motor3508.getVelocityRpm(2);
    auto DailVel = BSP::Motor::Dji::Motor2006.getVelocityRpm(1);
    auto Dail_pos = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);

    UpState();
    // 控制摩擦轮
    adrc_friction_L_vel.UpData(velL);
    adrc_friction_R_vel.UpData(velR);

    if (Now_Status_Serial == ONLY)
    {
        // 如果卡弹就让期望等于反馈
        if (JammingFMS.Get_Now_Status_Serial() == Jamming_Status::PROCESSING)
            Dail_target_pos = Dail_pos;

        pid_Dail_pos.setTarget(Dail_target_pos);
        pid_Dail_pos.GetPidPos(Kpid_Dail_pos, Dail_pos, 16384.0f);

        pid_Dail_vel.setTarget(pid_Dail_pos.getOut());
        pid_Dail_vel.GetPidPos(Kpid_Dail_vel, DailVel, 16384.0f);
    }
    else
    {
        // 拨盘速度控制
        pid_Dail_vel.GetPidPos(Kpid_Dail_vel, DailVel, 16384.0f);
        Dail_target_pos = Dail_pos;
    }

    target_Dail_torque = pid_Dail_vel.getOut();

    // 更新卡弹检测状态
    JammingFMS.UpState();

    // 根据卡弹检测结果应用控制策略
    // 如果堵转处理已经设置了目标力矩，这里不需要额外操作
    // 卡弹处理已经在JammingFSM::UpState中通过setTargetDailTorque方法修改力矩值

    // 拨盘发送
    //  Tools.vofaSend(adrc_Dail_vel.getZ1(), adrc_Dail_vel.getTarget(), adrc_Dail_vel.getFeedback(), 0, 0, 0);
    // 摩擦轮发送
    // Tools.vofaSend(adrc_friction_L_vel.getZ1(), adrc_friction_L_vel.getTarget(), adrc_friction_L_vel.getFeedback(),
    //                adrc_friction_R_vel.getZ1(), adrc_friction_R_vel.getTarget(), adrc_friction_R_vel.getFeedback());

    // // 火控
    // Tools.vofaSend(Heat_Limit.getFireNum(), Heat_Limit.getNowHeat(), Heat_Limit.getMaxHeat(), Heat_Limit.getCurSum(),
    // 0,
    //                0);

    CAN_Send();
}

void Class_ShootFSM::HeatLimit()
{
    auto CurL = BSP::Motor::Dji::Motor3508.getTorque(1);
    auto CurR = BSP::Motor::Dji::Motor3508.getTorque(2);

    auto velL = BSP::Motor::Dji::Motor3508.getVelocityRpm(1);
    auto velR = BSP::Motor::Dji::Motor3508.getVelocityRpm(2);

    Heat_Limit.setBoosterHeat(120, 40);
    Heat_Limit.setFrictionCurrent(CurL, CurR);
    Heat_Limit.setFrictionVel(velL, velR);
    // Heat_Limit.setTargetFire(20.0);

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
