#include "../APP/Variable.hpp"
#include "../HAL/My_HAL.hpp"
uint32_t Send_ms;

// 存储发送的数据
Motor_send_data_t msd_6020;
Motor_send_data_t msd_3508_2006;

// 设置电机数量，ID号
Dji_Motor_Data _Motor2006_[_Motor2006_SIZE] = {0};  uint8_t _Motor2006_ID_[_Motor2006_SIZE] = {0};
Dji_Motor_Data _Motor3508_[_Motor3508_SIZE] = {0};  uint8_t _Motor3508_ID_[_Motor3508_SIZE] = {1, 2, 3, 4};
Dji_Motor_Data _Motor6020_[_Motor6020_SIZE] = {0};  uint8_t _Motor6020_ID_[_Motor6020_SIZE] = {1, 2, 3, 4};

Dji_Motor Motor2006(0x200, _Motor2006_SIZE, _Motor2006_, _Motor2006_ID_);
Dji_Motor Motor3508(0x200, _Motor3508_SIZE, _Motor3508_, _Motor3508_ID_);
Dji_Motor Motor6020(0x204, _Motor6020_SIZE, _Motor6020_, _Motor6020_ID_);

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

