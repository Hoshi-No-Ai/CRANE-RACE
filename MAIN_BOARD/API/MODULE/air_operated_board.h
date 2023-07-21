#ifndef __AIR_OPERATED_BOARD_H__
#define __AIR_OPERATED_BOARD_H__

#include "stm32f4xx.h"
#include "string.h"
#include "flag_global.h"

// using _api_module_::g_uiAirValve;
// using _api_module_::g_uiAirValvePre;
// using _api_module_::g_usSwitch;
// using _api_module_::g_usSwitchPre;

//�������йغ궨��
#define CAN_SWITCH_ID 0x20
#define CAN_AIR_ID 0x30
#define CAN_AIR_ID_reply 0x32
#define CAN_SERVO_ID 0x40

// PWMͨ�����ƺ��������ı�־
#define Servo_Ctrl_Flag 0x01
#define DutedFan_Ctrl_Flag 0x02

class C_AirOperation {
   public:
    uint32_t uiAirValve;     //�����й�ȫ�ֱ���
    uint32_t uiAirValvePre;  //������һ��״̬
    uint16_t usSwitch;       //�����й�ȫ�ֱ���
    uint16_t usSwitchPre;    //������һʱ�̿���ֵ
    int Can_x;

    C_AirOperation() : Can_x(2){};
    C_AirOperation(int Cannum) : Can_x(Cannum){};
    ~C_AirOperation(){};

    void SendAirMsgByCan(void);
    void Send_LED_Mode(uint8_t mode);
    void SendServoMsgByCan(uint8_t chan, uint8_t value);  //ͨ��CAN���Ͷ����ռ�ձ�
    void SendServoMsgByCan_Plus(s16 value1, s16 value2, s16 value3, s16 value4);

    //�������йس�Ա����
    void open_value(int channel);
    void close_value(int channel);
    void change_value(int channel);
    bool is_value_open(int channel) { return (uiAirValve & (0x01 << (channel - 1))); }
    bool is_value_close(int channel) { return (!(uiAirValve & (0x01 << (channel - 1)))); }
    uint32_t get_value_value() { return uiAirValve; }
    void set_value_value(uint32_t value);

    //�����йس�Ա����
    void UpdateSwitchValue(CanRxMsg &RxMsg) { usSwitch = (uint16_t)RxMsg.Data[0]; }
    void Update_Switch(CanRxMsg &RxMsg);
    bool is_switch_on(int channel) { return (usSwitch & (0x0001 << channel)); }
    bool is_switch_off(int channel) { return (!(usSwitch & (0x0001 << channel))); }
    bool is_switch_change(int channel) {
        return ((usSwitch & (1 << channel)) != (usSwitchPre & (1 << channel)));
    }
    uint16_t get_switch_value() { return uiAirValve; }
};

extern class C_AirOperation AirOperation;

#endif
