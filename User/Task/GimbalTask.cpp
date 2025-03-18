#include "../Task/GimbalTask.hpp"
#include "../APP/KeyBroad.hpp"
#include "../APP/Mode.hpp"
#include "../APP/Tools.hpp"
#include "../Algorithm/PID.hpp"
#include "../Algorithm/Ude.hpp"
#include "../BSP/IMU/HI12H3_IMU.hpp"

#include "../BSP/Remote/Dbus.hpp"
#include "../Task/CommunicationTask.hpp"

// #include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/DM/DmMotor.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "cmsis_os2.h"

// #include "arm_math.h"

void GimbalTask(void *argument)
{
    osDelay(500);

    taskManager.addTask<Gimbal_Task>();

    for (;;)
    {
        taskManager.updateAll();
        osDelay(5);
    }
}

/**
 * @brief 底盘控制任务
 * @detail 实现移动底盘的状态控制逻辑
 */
class GimbalData
{
  public:
    // uint8_t DM_Run = 0; // 0: 初始状态, 1: On已执行, 2: Off已执行
    bool DM_On;

    float tar_pitch;
    float tar_yaw;

    float tar_shoot;
    float tar_dail_vel;
    int32_t tar_Dail_angle;

    float DM_Kp = 70;
    float DM_Kd = 1.7;
    float err = 0;
    float yaw_pos;
    float yaw_vel;

    bool is_Launch;

    uint8_t is_sin;
    float sin_hz;
    float sin_value;
	float B;

    float ff_value = 0;
    float ff_k = 0.13;
    float Yaw_final_out;

    float pitch_gravity_ff = -0.8; // pitch重力前馈
    float Pitch_final_out;

    uint32_t shoot_time_ms;
    uint32_t shoot_time;

    uint32_t send_ms;

    bool fire_flag;
    BSP::Motor::DM::DmRun DM_Run;
    BSP::Motor::DM::DmState DM_State = BSP::Motor::DM::DmState::OFF;

    void DMStateUpdata()
    {
        using namespace BSP::Motor;

        if (DM_Run == DM::DmRun::RUN_ON && DM_State == DM::DmState::OFF)
        {
            DM::Motor4310.On(&hcan2, 1);
            osDelay(10);
            DM_Run = DM::DmRun::NONE;
            DM_State = DM::DmState::ON;
        }
        else if (DM_Run == DM::DmRun::RUN_OFF && DM_State == DM::DmState::ON)
        {
            DM::Motor4310.Off(&hcan2, 1);
            osDelay(10);
            DM_Run = DM::DmRun::NONE;
            DM_State = DM::DmState::OFF;
        }
    }

    void setDmState(BSP::Motor::DM::DmRun run)
    {
        DM_Run = run;
    }

    void setYawTar(float yaw)
    {
        this->tar_yaw = yaw;
    }

    void setPitchTar(float pitch)
    {
        this->tar_pitch = pitch;
    }

    void setIsShoot(float shoot)
    {
        this->tar_shoot = shoot;
    }
};

GimbalData gimbal_data;

uint8_t state_num;

//=== 状态处理器实现 ===//
class Gimbal_Task::NormalHandler : public StateHandler
{
    Gimbal_Task &m_task;

  public:
    explicit NormalHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void handle() override
    {
        // 可访问m_task的私有成员进行底盘操作
        m_task.LaunchState(false);

        m_task.Gimbal();
    }
};

//=== 状态处理器实现 ===//
class Gimbal_Task::VisionHandler : public StateHandler
{
    Gimbal_Task &m_task;

  public:
    explicit VisionHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void VisionTar()
    {
        gimbal_data.sin_value = sinf(3.1415926 * 2 * gimbal_data.send_ms * gimbal_data.sin_hz * 0.005) * 45.0f;

        // pitch期望值计算
        gimbal_data.tar_pitch -= BSP::Remote::dr16.remoteRight().y * 0.1;
        gimbal_data.tar_pitch = Tools.clamp(gimbal_data.tar_pitch, 23, -5);

        // yaw期望值计算
        if (gimbal_data.is_sin == 0)
        {
            gimbal_data.tar_yaw -= BSP::Remote::dr16.remoteRight().x * 0.4;
        }
        else if (gimbal_data.is_sin == 1)
        {
            gimbal_data.tar_yaw = gimbal_data.sin_value + gimbal_data.B;
        }
    }

    void handle() override
    {
        // 可访问m_task的私有成员进行底盘操作
        state_num = 1;

        VisionTar();
        m_task.FilterUpdata();
        m_task.YawUpdata();
        m_task.PitchUpdata();
        m_task.Launch();
        m_task.CanSend();
    }
};

class Gimbal_Task::LaunchHandler : public StateHandler
{
  public:
    Gimbal_Task &m_task;

    explicit LaunchHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void handle() override
    {
        // 可访问m_task的私有成员进行底盘操作
        state_num = 2;
        m_task.LaunchState(true);

        m_task.Gimbal();
    }
};

float pitch;
float yaw;
class Gimbal_Task::KeyBoardHandler : public StateHandler
{
  public:
    Gimbal_Task &m_task;

    explicit KeyBoardHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void KeyBoardTar()
    {
        // pitch期望值计算
        auto mouse_vel = BSP::Remote::dr16.mouseVel();
        auto mouse_key = BSP::Remote::dr16.mouse();
        auto key = BSP::Remote::dr16.keyBoard();

        mouse_vel.x = Tools.clamp(mouse_vel.x, 50, -50);

        gimbal_data.tar_pitch += mouse_vel.y * 100;
        // yaw期望值计算
        gimbal_data.tar_yaw -= mouse_vel.x * 100;
        // pitch限幅
        gimbal_data.tar_pitch = Tools.clamp(gimbal_data.tar_pitch, 23, -5);

        APP::Key::keyBroad.UpKey(APP::Key::keyBroad.X, key.x);
        if (APP::Key::keyBroad.getFallingKey(APP::Key::keyBroad.X))
        {
            gimbal_data.tar_yaw -= 180.0f;
        }

        if (mouse_key.right)
        {
            //			auto Vision = Communicat::Vision_Data;
            gimbal_data.tar_pitch =
                (Communicat::Vision_Data.rx_target.pitch_angle + BSP::Motor::DM::Motor4310.getAngleDeg(1));
            gimbal_data.tar_yaw = Communicat::Vision_Data.rx_target.yaw_angle + BSP::IMU::imu.getAddYaw();
            //			gimbal_data.err =  tar_pitch.x1 - BSP::Motor::DM::Motor4310.getAngleDeg(1);

            if (mouse_key.left && gimbal_data.is_Launch == true)
                Communicat::Vision_Data.get_fire_num(&gimbal_data.tar_Dail_angle);
        }

        if (mouse_key.left && gimbal_data.is_Launch == true && mouse_key.right == 0)
        {
            //            gimbal_data.tar_dail_vel = -4500;
            gimbal_data.shoot_time = 100;
            gimbal_data.fire_flag = true;
        }
        else
        {
            gimbal_data.tar_dail_vel = 0;
            gimbal_data.shoot_time = 100000;
        }

        if (key.g)
        {

            //            gimbal_data.tar_shoot = 6000;
            gimbal_data.is_Launch = true;
            Gimbal_to_Chassis_Data.set_MCL(true);
        }

        if (key.b)
        {
            gimbal_data.tar_shoot = 0;
            gimbal_data.is_Launch = false;
            Gimbal_to_Chassis_Data.set_MCL(false);
        }
		static uint16_t num = 1;

		APP::Key::keyBroad.UpKey(APP::Key::keyBroad.C, key.c);
		num = Tools.clamp(num, 3, 1);
		Communicat::Vision_Data.setVisionMode(num);
		Gimbal_to_Chassis_Data.setVisionMode(num);
		if (APP::Key::keyBroad.getFallingKey(APP::Key::keyBroad.C))
		{
			num++;
			num %= 4;
		}
		
        m_task.LaunchState(gimbal_data.is_Launch);

        if (APP::Key::keyBroad.getFallingKey(APP::Key::keyBroad.Ctrl))
        {
            gimbal_data.DM_State = BSP::Motor::DM::DmState::OFF;
        }

        auto Vision = Communicat::Vision_Data;

        pitch = Vision.rx_target.pitch_angle + BSP::Motor::DM::Motor4310.getAngleDeg(1);
        //			pitch=BSP::Motor::DM::Motor4310.getAngleDeg(1);

        yaw = Vision.rx_target.yaw_angle + BSP::IMU::imu.getAddYaw();
        //					yaw=BSP::Motor::Dji::Motor6020.getAngleDeg(1);
    }

    void handle() override
    {
        state_num = 3;
        APP::Key::keyBroad.UpData();

        KeyBoardTar();
        m_task.FilterUpdata();
        m_task.YawUpdata();
        m_task.PitchUpdata();
        m_task.Launch();
        m_task.CanSend();
    }
};

uint8_t is_on;
float vel;
class Gimbal_Task::StopHandler : public StateHandler
{
    Gimbal_Task &m_task;

  public:
    explicit StopHandler(Gimbal_Task &task) : m_task(task)
    {
    }

    void handle() override
    {
        state_num = 4;

        m_task.LaunchState(false);
        m_task.Stop();
    }
};

//=== 任务方法实现 ===//
Gimbal_Task::Gimbal_Task()
{
    // 初始化默认状态
    updateState();
}

void Gimbal_Task::executeState()
{
    if (m_stateHandler)
    {
        m_stateHandler->handle();
    }
}

void Gimbal_Task::updateState()
{
    using namespace BSP;

    auto switch_right = Remote::dr16.switchRight();
    auto switch_left = Remote::dr16.switchLeft();

    if (Mode::Gimbal::Normal())
    {
        m_currentState = State::NormalState;
    }
    if (Mode::Gimbal::Vision())
    {
        m_currentState = State::VisionState;
    }
    if (Mode::Gimbal::Launch())
    {
        m_currentState = State::LaunchState;
    }
    if (Mode::Gimbal::KeyBoard())
    {
        m_currentState = State::KeyBoardState;
    }
    if (Mode::Gimbal::Stop())
    {
        m_currentState = State::StopState;
    }

    // 更新状态处理器
    switch (m_currentState)
    {
    case State::NormalState:
        m_stateHandler = std::make_unique<NormalHandler>(*this);
        break;
    case State::VisionState:
        m_stateHandler = std::make_unique<VisionHandler>(*this);
        break;
    case State::LaunchState:
        m_stateHandler = std::make_unique<LaunchHandler>(*this);
        break;
    case State::KeyBoardState:
        m_stateHandler = std::make_unique<KeyBoardHandler>(*this);
        break;
    case State::StopState:
        m_stateHandler = std::make_unique<StopHandler>(*this);
        break;
    }
}

TD tar_pitch(80);
TD tar_yaw(80);

TD tar_shoot(30);

TD Yaw_out(0);

TD shoot_vel_Left(100);
TD shoot_vel_Right(100);

Kpid_t Kpid_yaw_angle(10, 0, 0.0);
Kpid_t Kpid_yaw_vel(350, 0.0, 0.0);

PID pid_yaw_angle(4, 20);
PID pid_yaw_vel(0.0, 0.0);

Kpid_t Kpid_pitch_angle(0.0, 0.0, 0.0);
PID pid_pitch_angle(0.0, 0.0);

Ude yaw_ude(15, 0.12, 150, 100);
float tar;
void Gimbal_Task::TargetUpdata()
{
    gimbal_data.sin_value = cosf(3.1415926 * 2 * gimbal_data.send_ms * gimbal_data.sin_hz * 0.005) * gimbal_data.B;
    gimbal_data.send_ms++;
    // pitch期望值计算
    gimbal_data.tar_pitch -= BSP::Remote::dr16.remoteRight().y * 0.1;
    gimbal_data.tar_pitch = Tools.clamp(gimbal_data.tar_pitch, 23, -5);

    // yaw期望值计算
    if (gimbal_data.is_sin == 0)
    {
        gimbal_data.tar_yaw -= BSP::Remote::dr16.remoteRight().x * 0.7;
    }
    else if (gimbal_data.is_sin == 1)
    {
        gimbal_data.tar_yaw = gimbal_data.sin_value;
    }
    else
    {
        gimbal_data.tar_yaw = tar;
    }


    //    Tools.vofaSend(gimbal_data.tar_Dail_angle, 0, 0, 0, 0, 0);
}

void Gimbal_Task::FilterUpdata()
{
    tar_pitch.Calc(gimbal_data.tar_pitch);
    tar_yaw.Calc(gimbal_data.tar_yaw);
    // 摩擦轮期望值计算
    tar_shoot.Calc(gimbal_data.tar_shoot);

//    Yaw_vel.Calc(BSP::IMU::imu.getGyroZ());

    // 摩擦轮速度滤波
    shoot_vel_Left.Calc(BSP::Motor::Dji::Motor3508.getVelocityRpm(1));
    shoot_vel_Right.Calc(BSP::Motor::Dji::Motor3508.getVelocityRpm(2));
}


float x1, x2, x3, x1_d,x2_d, T = 0.005, l1, l2, l3, y,b = 0.6,u,wc = 120,u0;
float out,last_out;
float kp = 150, kd, p_out, tar1;
float step,err;
void LadrcDemo()
{
    auto yaw_angle = BSP::IMU::imu.getAddYaw();
	y = BSP::IMU::imu.getGyroZ();
//	l1 = 2*wc;
//	l2 = wc*wc;
//	
//	x1 += T*(x2 + l1*(y - x1) + b * u);
//	x2 += T*(l2*(y - last_out));
//	x3 
//	
//	last_out = x1;
    pid_yaw_angle.GetPidPos(Kpid_yaw_angle, tar_yaw.x1, yaw_angle, 16384);
	
	tar1 = pid_yaw_angle.getOut()+gimbal_data.ff_value;
//	p_out = kp * (tar1 - x1);
//	u0 = p_out;

//	u = u0 - x2/b;
	
	l1 = 2*wc;
	l2 = wc*wc;
//	l3 = wc * wc* wc;
	
    err = y - x1;
    x1 += (x2 + l1 * err + b * u) * T;
    x2 += l2 * err * T;
	
	last_out = x1;
	p_out = kp * (tar1 - x1);
	u0 = p_out;
	
	u = u0/b;
}

void Gimbal_Task::YawUpdata()
{
    auto yaw_angle = BSP::IMU::imu.getAddYaw();
    auto yaw_angle_vel = BSP::IMU::imu.getGyroZ();
	LadrcDemo();
    gimbal_data.ff_value = tar_yaw.x2 * gimbal_data.ff_k;

//    pid_yaw_angle.GetPidPos(Kpid_yaw_angle, tar_yaw.x1, yaw_angle, 16384);
    pid_yaw_vel.GetPidPos(Kpid_yaw_vel, tar_yaw.x1, BSP::IMU::imu.getGyroZ(), 16384);

//    yaw_ude.UdeCalc(yaw_angle_vel / 6.0f, pid_yaw_vel.getOut(), pid_yaw_angle.GetErr());
//	Yaw_out.Calc(pid_yaw_vel.getOut());
//    Tools.vofaSend(gimbal_data.tar_yaw, pid_yaw_vel.getOut() - yaw_ude.getCout(),
//    		BSP::IMU::imu.getAddYaw(), 0, 0, 0);
}

void Gimbal_Task::PitchUpdata()
{
    using namespace BSP::Motor;

    gimbal_data.Pitch_final_out = gimbal_data.pitch_gravity_ff;
    BSP::Motor::DM::Motor4310.ctrl_Motor(&hcan2, 1, tar_pitch.x1 * 0.0174532f, 0, gimbal_data.DM_Kp, gimbal_data.DM_Kd,
                                         gimbal_data.Pitch_final_out);

    gimbal_data.DMStateUpdata();
    gimbal_data.setDmState(BSP::Motor::DM::DmRun::RUN_ON);
}

Kpid_t Kpid_shoot_vel(5, 0.05, 0.003);
Kpid_t Kpid_Dial_vel(3, 0.0, 0);

PID pid_shoot_Left_vel(1000, 1000);
PID pid_shoot_Right_vel(1000, 1000);
PID pid_Dial_vel(0.0, 0.0);

Kpid_t Kpid_Dial_pos(3, 0, 0);
Kpid_t Kpid_Dial_pos_vel(200, 0, 0);

PID pid_Dial_pos_vel(0, 0);
PID pid_Dial_pos(0, 0);

uint16_t tar_shoot_vel = 0;

void Gimbal_Task::ShootUpdate()
{
    pid_shoot_Left_vel.GetPidPos(Kpid_shoot_vel, tar_shoot.x1, shoot_vel_Left.x1, 16384.0f);
    pid_shoot_Right_vel.GetPidPos(Kpid_shoot_vel, -tar_shoot.x1, shoot_vel_Right.x1, 16384.0f);
}

void Gimbal_Task::DialUpdata()
{
    auto mouse_key = BSP::Remote::dr16.mouse();

    // if (mouse_key.right)
    // {
    //     pid_Dial_pos.GetPidPos(Kpid_Dial_pos, gimbal_data.tar_Dail_angle,
    //     BSP::Motor::Dji::Motor2006.getAddAngleRad(1),
    //                            16384);
    //     pid_Dial_vel.GetPidPos(Kpid_Dial_pos_vel, pid_Dial_pos.getOut(),
    //     BSP::Motor::Dji::Motor2006.getVelocityRpm(1),
    //                            16384.0f);
    // }
    // else
    // {
    // pid_Dial_vel.GetPidPos(Kpid_Dial_vel, gimbal_data.tar_dail_vel, BSP::Motor::Dji::Motor2006.getVelocityRpm(1),
    //                        16384.0f);

    // 防止卡弹后继续增加累计值
    if (pid_Dial_pos.GetErr() > 80)
    {
        gimbal_data.tar_Dail_angle = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);
    }

    pid_Dial_pos.GetPidPos(Kpid_Dial_pos, gimbal_data.tar_Dail_angle, BSP::Motor::Dji::Motor2006.getAddAngleDeg(1),
                           16384.0f);
    pid_Dial_pos_vel.GetPidPos(Kpid_Dial_pos_vel, pid_Dial_pos.getOut(), BSP::Motor::Dji::Motor2006.getVelocityRpm(1),
                               16384.0f);
    // }
}

void Gimbal_Task::CanSend()
{
    gimbal_data.Yaw_final_out = Tools.clamp(u, 16384.0f, -16384.0f);
    BSP::Motor::Dji::Motor6020.setCAN(gimbal_data.Yaw_final_out, 2);

    BSP::Motor::Dji::Motor3508.setCAN(pid_shoot_Left_vel.getOut(), 2);
    BSP::Motor::Dji::Motor3508.setCAN(pid_shoot_Right_vel.getOut(), 3);

    BSP::Motor::Dji::Motor2006.setCAN(pid_Dial_pos_vel.getOut(), 1);

    BSP::Motor::Dji::Motor6020.sendCAN(&hcan1, 0);
    BSP::Motor::Dji::Motor2006.sendCAN(&hcan1, 0);
    BSP::Motor::Dji::Motor3508.sendCAN(&hcan1, 1);

//    Tools.vofaSend(u, x1,
//    		BSP::IMU::imu.getGyroZ(), tar1, BSP::IMU::imu.getAddYaw(), tar_yaw.x1);
}

void Gimbal_Task::Stop()
{
    // 达妙失能
    TargetUpdata();
    FilterUpdata();
    gimbal_data.DMStateUpdata();
    gimbal_data.setDmState(BSP::Motor::DM::DmRun::RUN_OFF);

    gimbal_data.tar_yaw = BSP::IMU::imu.getAddYaw();
    gimbal_data.tar_pitch = BSP::Motor::DM::Motor4310.getAngleDeg(1);

    YawUpdata();
    //	PitchUpdata();

    pid_yaw_angle.clearPID();
    pid_yaw_vel.clearPID();
    pid_Dial_pos_vel.clearPID();
    yaw_ude.clear();
	
	u = 0;

    Launch();

    CanSend();
}

void Gimbal_Task::Gimbal()
{
    TargetUpdata();
    FilterUpdata();
    YawUpdata();
    PitchUpdata();

    Launch();
    CanSend();
}

void Gimbal_Task::Launch()
{
    ShootUpdate();
    DialUpdata();
}

void Gimbal_Task::LaunchState(bool is_Launch)
{
    if (is_Launch == true)
    {
        int32_t shoot_time_tick = HAL_GetTick();
        auto sw = BSP::Remote::dr16.sw();
        auto mouse_key = BSP::Remote::dr16.mouse();

        //        // 弹速控制
        if (sw >= 1)
        {
            gimbal_data.shoot_time = 50;
            gimbal_data.fire_flag = true;
        }
        else if (sw >= 0.6 || mouse_key.left == 1)
        {
            gimbal_data.shoot_time = 100;
            gimbal_data.fire_flag = true;
        }
        else if (sw >= 0.3)
        {
            gimbal_data.shoot_time = 200;
            gimbal_data.fire_flag = true;
        }
        else
        {
            gimbal_data.fire_flag = false;
        }

        if (gimbal_data.fire_flag == true && shoot_time_tick - gimbal_data.shoot_time_ms >= gimbal_data.shoot_time)
        {
            gimbal_data.tar_Dail_angle -= 40; // 8191 * 减速比(36) / 弹舱载弹数
            gimbal_data.shoot_time_ms = shoot_time_tick;
        }

        //        gimbal_data.tar_dail_vel = BSP::Remote::dr16.sw() * 3000;
        gimbal_data.tar_shoot = 6000;
    }
    else if (is_Launch == false)
    {
        gimbal_data.tar_Dail_angle = BSP::Motor::Dji::Motor2006.getAddAngleDeg(1);
        gimbal_data.tar_dail_vel = 0;
        gimbal_data.tar_shoot = 0;
    }
}
