#pragma once

class RM_Key
{
public:
    //��ǰ״̬����һ��״̬��������״̬���½���״̬
    bool NowKey,lastKey,RisingEdge,FallingEdge;
    void UpKey(bool key);//�����ź�
    bool GetRisingKey();//��ȡ�������ź�
    bool GetFallingKey();//��ȡ�½����ź�
};
inline void RM_Key::UpKey(bool key)
{
    this->lastKey = this->NowKey;//������һ���ź�
    this->NowKey = key;//��ȡ��ǰ�ź�
    this->RisingEdge = this->FallingEdge = false;//���״̬
    if(this->NowKey - this->lastKey == 1)this->FallingEdge = true;//�����������ź�
    if(this->lastKey - this->NowKey == 1)this->RisingEdge = true;//�����½����ź�
}

inline bool RM_Key::GetRisingKey()
{
    return this->RisingEdge;//�������ź�
}

inline bool RM_Key::GetFallingKey()
{
    return this->FallingEdge;//�½����ź�
}
