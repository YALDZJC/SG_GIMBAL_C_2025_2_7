#include "../APP/Variable.hpp"
#include "../HAL/My_HAL.hpp"
uint32_t Send_ms;



// PID角度环设置
Kpid_t Kpid_6020_angle(0.4, 0, 0);
PID pid_angle_String[4];

// PID角度环设置
Kpid_t Kpid_6020_vel(100, 0, 0);
PID pid_vel_String[4];

Kpid_t ude_Kpid_angle(0.2, 0, 0);
PID ude_angle_demo;
Kpid_t ude_Kpid_vel(15, 0, 0);
PID ude_vel_demo;

// PID速度环设置
Kpid_t Kpid_3508_vel(5, 0.2, 0);
PID pid_vel_Wheel[4] = {
    {1000, 8000},
    {1000, 8000},
    {1000, 8000},
    {1000, 8000},
};


// 3508速度滤波
TD td_3508_speed[4] = {
    {300},
    {300},
    {300},
    {300},
};
// 力矩滤波
TD td6020_torque(100);
TD td3508_torque(100);

// 期望值滤波
TD tar_vw(30);
TD tar_vx(30);
TD tar_vy(30);
TD td_FF_Tar(100);

TD td_Power[4] = {
    {600},
    {600},
    {600},
    {600},
};

// 前馈
FeedTar feed_6020[4] = {
    {50, 5},
    {50, 5},
    {50, 5},
    {50, 5},
};

FeedTar feed_6020_1(50, 5);
FeedTar feed_6020_2(50, 5);
FeedTar feed_6020_3(50, 5);
FeedTar feed_6020_4(50, 5);





// 创建底盘变量实例
Chassis_Data_t Chassis_Data;

