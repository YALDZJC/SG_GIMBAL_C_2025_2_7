#include "../Task/EvenTask.hpp"
#ifdef ADRC_TEST
#include "../Algorithm/LADRC/Adrc.hpp"
#endif
#include "../APP/Buzzer.hpp"
#include "../APP/LED.hpp"
#include "../APP/Variable.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Remote/Dbus/Dbus.hpp"

#include "../BSP/Motor/Dji/DjiMotor.hpp"

#include "../BSP/CAN/Bsp_Can.hpp"
#include "../BSP/Init.hpp"
#include "cmsis_os2.h"
#include "tim.h"

// using namespace Event;
Dir Dir_Event;

auto LED_Event = std::make_unique<LED>(&Dir_Event); // 让LED灯先订阅，先亮灯再更新蜂鸣器
auto Buzzer_Event = std::make_unique<Buzzer>(&Dir_Event);

void EventTask(void *argument)
{
    osDelay(500);

    for (;;)
    {
        Dir_Event.UpEvent();
        osDelay(1);

    }
}

bool Dir::Dir_Remote()
{
    bool Dir = BSP::Remote::dr16.ISDir();

    DirData.Dr16 = Dir;

    return Dir;
}

bool Dir::Dir_Yaw()
{
    bool Dir = BSP::Motor::Dji::Motor6020.ISDir();
    DirData.Yaw = BSP::Motor::Dji::Motor6020.GetDir(1);


    return DirData.Yaw;
}

bool Dir::Dir_Pitch()
{
    bool Dir = BSP::Motor::Dji::Motor3508.ISDir();

    // for (int i = 0; i < 4; i++)
    // {
    //     DirData.Wheel[i] = BSP::Motor::Dji::Motor3508.GetDir(i);
    // }

    return Dir;
}

bool Dir::Init_Flag()
{
    DirData.InitFlag = InitFlag;
    return InitFlag;
}

bool Dir::Dir_IMU()
{
    bool Dir = BSP::IMU::imu.ISDir();

    DirData.Imu = Dir;

    BSP::IMU::imu.Init();

    return Dir;
}

/**
 * @brief 更新事件
 *
 */
void Dir::UpEvent()
{
    Init_Flag();

    Dir_Remote();
    Dir_Pitch();
    Dir_Yaw();
    Dir_IMU();

    Notify();
}
