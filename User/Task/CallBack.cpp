#include "../Task/CallBack.hpp"
#include "../HAL/My_HAL.hpp"
#include "../APP/Variable.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"
#include "../BSP/Remote/Dbus.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"

//can_filo0中断接收
CAN_RxHeaderTypeDef RxHeader;	//can接收数据
uint8_t RxHeaderData[8] = { 0 };

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//接受信息 
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,RxHeaderData);
	if(hcan == &hcan1)
	{
		// Motor3508.Parse(RxHeader,RxHeaderData);
		// Motor6020.Parse(RxHeader,RxHeaderData);		
		BSP::Motor::DM::Motor4310.Parse(RxHeader,RxHeaderData);
	}
	if(hcan == &hcan2)
	{

	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	Remote::dr16.Parse(huart, Size);
	BSP::IMU::imu.Parse(huart, Size);
}


