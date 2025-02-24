#include "../BSP/Init.hpp"
#include "../BSP/Dbus.hpp"
#include "../BSP/HI12H3_IMU.hpp"
#include "../HAL/My_HAL.hpp"
#include "../Task/CallBack.hpp"

#include "tim.h"

bool InitFlag = false;
void Init()
{
    // 初始化
    Remote::dr16.Init();

    CAN::BSP::Can_Init();
    BSP::IMU::imu.Init();

    HAL_TIM_Base_Start_IT(&htim7);

    // ?????
    HAL_TIM_Base_Start(&htim4);
    // ??PWM??
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

    InitFlag = true;
}
