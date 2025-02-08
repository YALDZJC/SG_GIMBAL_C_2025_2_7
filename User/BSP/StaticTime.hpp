#pragma once
#include "stm32f4xx_hal.h"
#include "stdxxx.hpp"
#include "Key.hpp"

class RM_StaticTime
{
public:
	uint32_t lastTime;	//��һʱ��
	RM_Key key;			//�ź���
	void UpLastTime();//������һʱ��
	bool ISOne(uint32_t targetTime);//�жϵ����ź�
	bool ISGL(uint32_t targetTime, uint8_t percentage = 50/*�ٷֱ�ռ��*/);//�ж������ź�
	bool ISDir(uint32_t dirTime);//��ʱ������
	bool ISFromOne(uint64_t nowTime, uint64_t targetTime);//�Զ����жϵ����ź�
	bool ISFromGL(uint64_t nowTime, uint64_t targetTime, uint8_t percentage = 50/*�ٷֱ�ռ��*/);//�Զ����ж������ź�
};

inline void RM_StaticTime::UpLastTime()
{
	this->lastTime = HAL_GetTick();
}

inline bool RM_StaticTime::ISOne(uint32_t targetTime)
{
	this->key.UpKey(HAL_GetTick() % targetTime);//�������״̬
	if (this->key.GetRisingKey()) return true;

	return false;
}

inline bool RM_StaticTime::ISGL(uint32_t targetTime, uint8_t percentage)
{
	this->key.UpKey((HAL_GetTick() % targetTime / (float)targetTime) * 100 > 100 - percentage);//�������״̬
	return this->key.NowKey;
}

inline bool RM_StaticTime::ISDir(uint32_t dirTime)
{
	if(HAL_GetTick() - this->lastTime >= dirTime)
		return true;
		
  	return false;
}

inline bool RM_StaticTime::ISFromOne(uint64_t nowTime, uint64_t targetTime)
{
	this->key.UpKey(nowTime % targetTime);//�������״̬
	if (this->key.GetRisingKey())return true;
	return false;
}

inline bool RM_StaticTime::ISFromGL(uint64_t nowTime, uint64_t targetTime, uint8_t percentage)
{
	this->key.UpKey((nowTime % targetTime / (float)targetTime) * 100 > 100 - percentage);//�������״̬
	return this->key.NowKey;
}



