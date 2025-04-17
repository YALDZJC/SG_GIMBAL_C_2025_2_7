#include "../Task/CommunicationTask.hpp"
#include "../Task/ShootTask.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "cmsis_os2.h"
float hz_send;

void ShootTask(void *argument)
{
    for (;;)
    {
        // hz_send += 0.001;
        // Communicat::Vision_Data.time_demo();

        osDelay(1);
    }
}

namespace TASK::Shoot
{

void CAN_Set(void)
{
    auto Motor_Friction = BSP::Motor::Dji::Motor3508;
    auto Motor_Dail = BSP::Motor::Dji::Motor2006;

    // Motor_Friction.setCAN(&hcan1, 1);
    // Motor_Dail.setCAN(&hcan1, 1);
}

void CAN_Send(void)
{
    auto Motor_Friction = BSP::Motor::Dji::Motor3508;
    auto Motor_Dail = BSP::Motor::Dji::Motor2006;

    Motor_Friction.sendCAN(&hcan1, 1);
    Motor_Dail.sendCAN(&hcan1, 1);
}   
}
