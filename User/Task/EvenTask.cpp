#include "../Task/EvenTask.hpp"
#include "../APP/Variable.hpp"
#include "../APP/Buzzer.hpp"
#include "../APP/LED.hpp"
#include "../BSP/Init.hpp"

#include "cmsis_os2.h"
#include "tim.h"
// using namespace Event;



void EventTask(void *argument)
{
//	Dir Dir_Event;

//	  LED LED_Event{&Dir_Event}; // 让LED灯先订阅，先亮灯再更新蜂鸣器
//    Buzzer Buzzer_Event{&Dir_Event};

    osDelay(500);

    for (;;)
    {
//        Dir_Event.Notify();

        osDelay(1);
    }
}
bool Dir::Dir_Remote()
{
    bool Dir = dr16.ISDir();

    DirData.Dr16 = Dir;

    return Dir;
}

bool Dir::Dir_String()
{
    bool Dir = Motor6020.ISDir();

    for (int i = 0; i < 4; i++)
    {
        DirData.String[i] = Motor6020.GetDir(Get_InitID_6020(i));
    }

    return Dir;
}

bool Dir::Dir_Wheel()
{
    bool Dir = Motor3508.ISDir();

    for (int i = 0; i < 4; i++)
    {
        DirData.Wheel[i] = Motor3508.GetDir(Get_InitID_3508(i));
    }

    return Dir;
}

bool Dir::Init_Flag()
{
    DirData.InitFlag = InitFlag;
    return InitFlag;
}

/**
 * @brief 更新事件
 *
 */
void Dir::UpEvent()
{
    Dir_Remote();
    Dir_String();
    Dir_Wheel();
    Init_Flag();
}
