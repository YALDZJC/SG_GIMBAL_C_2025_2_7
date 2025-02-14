#include "../Task/CallBack.hpp"
#include "../BSP/Init.hpp"
#include "../HAL/My_HAL.hpp"
#include "../BSP/dr16.hpp"
#include "../BSP/HI12H3_IMU.hpp"

#include "tim.h"

bool InitFlag = false;
void Init()
{
     dr16.Init();
     Can_Init();
     IMU::imu.Init();
     
     HAL_TIM_Base_Start_IT(&htim7);

     // ?????
     HAL_TIM_Base_Start(&htim4);
     // ??PWM??
     HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

     HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
     HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
     HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, pData1, sizeof(pData1));

		InitFlag = true;
}
