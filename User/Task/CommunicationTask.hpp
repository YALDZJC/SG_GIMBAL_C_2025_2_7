#pragma once

#include "../BSP/stdxxx.hpp"
#include "../BSP/dr16.hpp"
#include "../Task/EvenTask.hpp"

#define L_MODE1 (RM_Clicker::RC_Ctl.rc.s1 == 1) // 底盘动作
#define L_MODE2 (RM_Clicker::RC_Ctl.rc.s1 == 3) // 底盘动作
#define L_MODE3 (RM_Clicker::RC_Ctl.rc.s1 == 2) // 底盘动作

#define R_MODE1 (RM_Clicker::RC_Ctl.rc.s2 == 1) // 云台动作
#define R_MODE2 (RM_Clicker::RC_Ctl.rc.s2 == 3) // 云台动作
#define R_MODE3 (RM_Clicker::RC_Ctl.rc.s2 == 2) // 云台动作

/*遥控器信号源切换*/
/*
	CONTROL_SIG 0 遥控器
	CONTROL_SIG 1 上下板
*/
#define CONTROL_SIG 0
#if CONTROL_SIG == 0
// 模式切换
#define Universal ((L_MODE1 && R_MODE1) || (L_MODE1 && R_MODE2) || (L_MODE1 && R_MODE3)) // （1）万向模式
#define Follow (L_MODE2 && R_MODE1) || (L_MODE2 && R_MODE3)								 // （2）底盘跟随
#define Rotating (L_MODE3 && R_MODE1) || (L_MODE3 && R_MODE2)							 // （3）小陀螺
#define KeyBoard (L_MODE2 && R_MODE2)													 // （4键鼠模式）
#define Stop (L_MODE3 && R_MODE3)							 // （5停止模式）

// 期望值切换
#define TAR_LX RC_LX
#define TAR_LY RC_LY
#define TAR_RX RC_RX
#define TAR_RY RC_RY

#elif CONTROL_SIG == 1
// 模式切换
#define Universal (Gimbal_to_Chassis_Data.Universal_t) // （1）万向模式
#define Follow (Gimbal_to_Chassis_Data.Follow_t)	   // （2）底盘跟随
#define Rotating (Gimbal_to_Chassis_Data.Rotating_t)   // （3）小陀螺
// #define KeyBoard (Gimbal_to_Chassis_Data.KeyBoard_t)											 // （4键鼠模式）
#define Stop (Gimbal_to_Chassis_Data.stop)			   // （5停止模式）

// 期望值切换
#define TAR_LX Gimbal_to_Chassis_Data.int16_RC_LX
#define TAR_LY Gimbal_to_Chassis_Data.int16_RC_LY
#define TAR_RX Gimbal_to_Chassis_Data.int16_RC_RX
#define TAR_RY Gimbal_to_Chassis_Data.int16_RC_RY
#endif

namespace Communicat
{
	class Gimbal_to_Chassis
	{
	public:
		void Data_send();

	private:
		uint8_t head = 0xA5;//帧头

		struct __attribute__((packed)) Direction // 方向结构体
		{
			uint8_t LX;
			uint8_t LY;

			uint8_t Rotating_vel;
			uint16_t Yaw_encoder_angle_err;
		};

		struct __attribute__((packed)) ChassisMode // 底盘模式
		{
			uint8_t Universal_mode : 1;
			uint8_t Follow_mode : 1;
			uint8_t Rotating_mode : 1;
			uint8_t stop : 1;
		};

		struct __attribute__((packed)) UiList // 底盘模式
		{
			uint8_t MCL : 1;
			uint8_t CM : 1;
			uint8_t BP : 1;
		};
		
		Direction direction;
		ChassisMode chassis_mode;
		UiList ui_list;
	};
	
	inline uint8_t getSendRc(uint16_t RcData)
	{
		return (RcData / 6) - 110;
	}
}

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif

	void CommunicationTask(void *argument);

#ifdef __cplusplus
}
#endif
