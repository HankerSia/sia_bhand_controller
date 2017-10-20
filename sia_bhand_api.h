//============================================================================
// Name        : sia_bhand_api.h
// Author      : HK, Shenyang Institute of Automation,CAS.
// Version     : 1.0
// Copyright   : BSD
// Description : A ROS driver for controlling the Barett robotic manipulator hand
// Created on  : Apr 27, 2017,Maintainer email="susthekai@qq.com"
//============================================================================
#ifndef _SIA_BHAND_API_H
#define _SIA_BHAND_API_H
#include <stdint.h>
#include "string"

using namespace std;

//bhand velocity control:globals variable
//PuckID
extern int FINGER1;
extern int FINGER2;
extern int FINGER3;
extern int SPREAD;

extern int BASE_TYPE;
extern int TIP_TYPE;
extern int SPREAD_TYPE;
extern float BASE_LIMIT;
extern float TIP_LIMIT;
extern float SPREAD_LIMIT;

extern float MAX_ENCODER_TICKS;
extern float MAX_SPREAD_TICKS;
extern float MAX_FINGERTIP_TICKS;

extern int V;
extern int TSTOP;
extern int MODE;
extern int MODE_VEL;

extern uint32_t F1_POSITION;
extern uint32_t F2_POSITION;
extern uint32_t F3_POSITION;
extern uint32_t SPREAD_POSITION;

extern int AxisForceQuery;
extern int AxisTorqueQuery;
extern uint32_t axisForce;
extern uint32_t axisTorque;

typedef DWORD   (*funCAN_Init_TYPE)(HANDLE hHandle, WORD wBTR0BTR1, int nCANMsgType);
typedef HANDLE  (*funLINUX_CAN_Open_TYPE)(const char *szDeviceName, int nFlag);
typedef DWORD   (*funCAN_VersionInfo_TYPE)(HANDLE hHandle, LPSTR lpszTextBuff);
typedef DWORD   (*funCAN_Close_TYPE)(HANDLE hHandle);
typedef DWORD   (*funLINUX_CAN_Read_TYPE)(HANDLE hHandle, TPCANRdMsg* pMsgBuff);
typedef DWORD   (*funCAN_Status_TYPE)(HANDLE hHandle);
typedef int     (*funnGetLastError_TYPE)(void);
typedef DWORD   (*funCAN_Write_TYPE)(HANDLE hHandle, TPCANMsg* pMsgBuff);

struct axisSensorData
{
    float force_x;
    float force_y;
    float force_z;
    float torque_x;
    float torque_y;
    float torque_z;
};

class pcan_operate
{
public:
    pcan_operate();

    bool pcan_init(string dev_name);
    int read_loop(bool display_on);
    DWORD send_msg(int msgID, unsigned int data[], int len, unsigned int delay_us);
    void check_error(__u32 result, string location_of_error);
    void print_message(TPCANMsg *m);
    void print_message_ex(TPCANRdMsg *mr);
    ~pcan_operate();

public:
    void *lib_handle = NULL;//define pointer used for file acess of libpcan.so
    funCAN_Init_TYPE        funCAN_Init;
    funLINUX_CAN_Open_TYPE  funLINUX_CAN_Open;
    funCAN_VersionInfo_TYPE funCAN_VersionInfo;
    funCAN_Close_TYPE       funCAN_Close;
    funLINUX_CAN_Read_TYPE  funLINUX_CAN_Read;
    funCAN_Status_TYPE      funCAN_Status;
    funnGetLastError_TYPE   funnGetLastError;
    funCAN_Write_TYPE       funCAN_Write;

    HANDLE pcan_handle =NULL;//void *pcan_handle
};

class bhand
{
public:
    bhand(string dev_name);
    ~bhand();
    DWORD send_msg(int msgID,unsigned int data[],int len,unsigned int delay_us=0);
    void barett_hand_init();

    void setJointPosition(float f1_pos_f,float f2_pos_f,float f3_pos_f,float sp_pos_f);
    //bhand velocity control:function declaration
    int rad_to_enc(float rad,int type=BASE_TYPE);
    float enc_to_rad(uint32_t enc,int type=BASE_TYPE);
    void check_error(DWORD result,string location_of_error);

    void set_16(int msgID,int propID,int tmp_value);
    void set_32(int msgID,int propID,int tmp_value);
    void set_property(int msgID,int propID,int value);
    void set_velocity(int puckID,int velocity);
    void setJointVelocity(int finger,float rad_s);
    uint32_t get_position(int msgID,int depth=1);
    float get_velocity(int msgID);
    bool wait_act_finish();

    //6 axis force handle
    void axisForce_init();
    int wavePuck(int bus,int id);
    int compile(
            int property        /** The property being compiled (use the enumerations in btcan.h) */,
            long longVal        /** The value to set the property to */,
            unsigned char *data /** A pointer to a character buffer in which to build the data payload */,
            int *dataLen        /** A pointer to the total length of the data payload for this packet */,
            int isForSafety        /** A flag indicating whether this packet is destined for the safety circuit or not */);
    int setPropertySlow(int bus, int id, int property, int verify, long value);
    axisSensorData get_AxisForce(int force_id,int depth=1);
    void displayAxisSensordata (axisSensorData axisData);
    void displayAxisForceData (axisSensorData axisData);
    void displayAxisTorqueData (axisSensorData axisData);
    axisSensorData get_AxisTorque(int torque_id,int depth=1);
public:
    pcan_operate can_handle;
    HANDLE pcan_handle =NULL;//void *pcan_handle
    axisSensorData axisSensor;
};





#endif
