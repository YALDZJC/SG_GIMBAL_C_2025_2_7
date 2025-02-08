#include "../BSP/Init.hpp"
#include "../HAL/My_HAL.hpp"
#include "../BSP/dr16.hpp"

#include "tim.h"

bool InitFlag = false;
void Init()
{
     dr16.Init();
     Can_Init();

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
