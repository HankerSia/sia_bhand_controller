//============================================================================
// Name        : sia_bhand_api.cpp
// Author      : HK, Shenyang Institute of Automation,CAS.
// Version     : 1.0
// Copyright   : BSD
// Description : A ROS driver for controlling the Barett robotic manipulator hand
// Created on  : Apr 27, 2017,Maintainer email="susthekai@qq.com"
//============================================================================
#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <dlfcn.h>
#include <libpcan.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include "string"
#include "sia_bhand_api.h"
using namespace std;
//default name of pcan
#define DEFAULT_NODE "/dev/pcan0"
//6 axis force sensor
#define FTSENSORID (8)
#define FTDATAPROPERTY (54)
//bhand velocity control:globals variable
//PuckID
int FINGER1 = 11;
int FINGER2 = 12;
int FINGER3 = 13;
int SPREAD = 14;

int BASE_TYPE = 0;
int TIP_TYPE = 1;
int SPREAD_TYPE = 2;
float BASE_LIMIT = 140.0;
float TIP_LIMIT = 48.0;
float SPREAD_LIMIT = 180.0;

float MAX_ENCODER_TICKS=195000.0;
float MAX_SPREAD_TICKS=36000.0;
float MAX_FINGERTIP_TICKS=78000.0;

int V = 44;
int TSTOP = 78;
int MODE = 8;
int MODE_VEL=4;

uint32_t F1_POSITION = 0x166;
uint32_t F2_POSITION = 0x186;
uint32_t F3_POSITION = 0x1a6;
uint32_t SPREAD_POSITION = 0x1c6;

int AxisForceQuery=8;
int AxisTorqueQuery=8;
uint32_t AxisForce = 0x50a;
uint32_t AxisTorque= 0x50b;

/*
Notes : Do some init process,such as Mapping pcan function
Input : None.
Output: None.
*/
pcan_operate::pcan_operate()
{
    lib_handle = dlopen("libpcan.so", RTLD_LAZY );
    if (!lib_handle){
        printf("Open Error:%s.\n",dlerror());//if file can't be loaded,return null,get reason using dlerror function
    }

    char *errorInfo;//error information pointer
    //one-to-one mapping using dlsym function,if return null,mapping would be failed
    funCAN_Init         =(funCAN_Init_TYPE)         dlsym(lib_handle,"CAN_Init");
    funLINUX_CAN_Open   =(funLINUX_CAN_Open_TYPE)   dlsym(lib_handle,"LINUX_CAN_Open");
    funCAN_Close        =(funCAN_Close_TYPE)        dlsym(lib_handle,"CAN_Close");
    funCAN_VersionInfo  =(funCAN_VersionInfo_TYPE)  dlsym(lib_handle,"CAN_VersionInfo");
    funLINUX_CAN_Read   =(funLINUX_CAN_Read_TYPE)   dlsym(lib_handle,"LINUX_CAN_Read");
    funCAN_Status       =(funCAN_Status_TYPE)       dlsym(lib_handle,"CAN_Status");
    funnGetLastError    =(funnGetLastError_TYPE)    dlsym(lib_handle,"nGetLastError");
    funCAN_Write        =(funCAN_Write_TYPE)        dlsym(lib_handle,"CAN_Write");

    errorInfo = dlerror();//get error using dlerror function,and clear the error list in memory
    if (errorInfo != NULL){
        printf("Dlsym Error:%s.\n",errorInfo);
    }
}

/*
Notes : Get handle of USB-pcan,and set some parameter of CAN bus.
Input : the device name,you can get it use commands "ls /dev/pcan*".the default name:"/dev/pcan0".
Output: None.
*/
bool pcan_operate::pcan_init(string dev_name)
{
    char txt[VERSIONSTRING_LEN];            //store information of can version
    unsigned short wBTR0BTR1 = CAN_BAUD_1M; //set the communicate baud rate of can bus
    int nExtended = CAN_INIT_TYPE_ST;       //set can message int standard model
    const char  *szDevNode = DEFAULT_NODE;  //define const pointer point to device name

    if(dev_name.c_str() !="")
        pcan_handle =funLINUX_CAN_Open(dev_name.c_str(), O_RDWR | O_NONBLOCK);
    else
    {
        pcan_handle =funLINUX_CAN_Open(szDevNode, O_RDWR | O_NONBLOCK);
        dev_name=DEFAULT_NODE;
    }

    //judge whether the call is success.if pcan_handle=null,the call would be failed
    if(pcan_handle){
        printf("CAN Bus test: %s have been opened\n", szDevNode);
        errno = funCAN_VersionInfo(pcan_handle, txt);
        if (!errno)
            printf("CAN Bus test: driver version = %s\n", txt);
        else {
            perror("CAN Bus test: CAN_VersionInfo()");
        }
        if (wBTR0BTR1) {
                errno = funCAN_Init(pcan_handle, wBTR0BTR1, nExtended);
                if (errno) {
                    perror("CAN Bus test: CAN_Init()");
                }
                else
                    printf("Device Info: %s; CAN_BAUD_1M; CAN_INIT_TYPE_ST\n", szDevNode);
            }
    }
    else
        printf("CAN Bus test: can't open %s\n", szDevNode);
}

/*
Notes : read data of can bus, you can add extra data monitor process in this function.
Input : when display_on=true, this function will print all the data read on can bus.
Output: none.
*/
int pcan_operate::read_loop(bool display_on)
{

    TPCANRdMsg m;
    __u32 status;

    if (funLINUX_CAN_Read(pcan_handle, &m)) {
        //perror("receivetest: LINUX_CAN_Read()");
        return errno;
    }

    // check if a CAN status is pending
    if (m.Msg.MSGTYPE & MSGTYPE_STATUS) {
        status = funCAN_Status(pcan_handle);
        if ((int)status < 0) {
            errno = funnGetLastError();
            perror("receivetest: CAN_Status()");
            return errno;
        }

        printf("receivetest: pending CAN status 0x%04x read.\n",
               (__u16)status);
    }

    if(display_on)
    {
        print_message_ex(&m);
    }
    //do extra processing if you need. such as monitor other data on can bus.

}

/*
 Notes : use CAN to send a message
 Input : CAN id, data array, length of the data, delay some micro-seconds when data is sended.
Output : the state of CAN write.
Example: unsigned int data[]={0x30};
       : send_msg(msgID+0xc0,data,sizeof(data)/sizeof(data[0]),9000);
*/
DWORD pcan_operate::send_msg(int msgID, unsigned int data[], int len, unsigned int delay_us)
{
    TPCANMsg msg;
    msg.LEN =len;
    msg.MSGTYPE =MSGTYPE_STANDARD;
    for(int i=0;i<len;i++)
    {
        msg.DATA[i]=data[i];
    }
    msg.ID = msgID;
    DWORD stat=funCAN_Write(pcan_handle,&msg);
    check_error(stat,"write");
    //print_message(&msg);
    usleep(delay_us);
    return stat;
}

/*
Notes : check whether the can send/write/...  is success or not.
Input : state of can handle,description.
Output: none.
*/
void pcan_operate::check_error(__u32 result, string location_of_error)
{
    if(result==CAN_ERR_OK)
        return;
    else
    {
        printf("CAN Error Number:%x, while attempting to %s ",result,location_of_error.c_str());
    }
}

/*
Notes : print a can format message.
Input : meaasge.
Output: none.
*/
void pcan_operate::print_message(TPCANMsg *m)
{
    int i;

    //print RTR, 11 or 29, CAN-Id and datalength
    printf("receivetest: %c %c 0x%08x %1d ",
            ((m->MSGTYPE & MSGTYPE_RTR) ? 'r' : 'm') -
                ((m->MSGTYPE ) ? 0x20 : 0),
            (m->MSGTYPE & MSGTYPE_EXTENDED) ? 'e' : 's',
             m->ID,
             m->LEN);

    //don't print any telegram contents for remote frames
    if (!(m->MSGTYPE & MSGTYPE_RTR))
        for (i = 0; i < m->LEN; i++)
            printf("%02x ", m->DATA[i]);
          //printf("%3d ", m->DATA[i]);//decimal format print.
    printf("\n");
}

/*
Notes : print a can message read on can bus.
Input : can message read on can bus.
Output: none.
*/
void pcan_operate::print_message_ex(TPCANRdMsg *mr)
{
    printf("%u.%3u ", mr->dwTime, mr->wUsec);
    print_message(&mr->Msg);
}

pcan_operate::~pcan_operate()
{
    funCAN_Close(pcan_handle);
    dlclose(lib_handle);

}


/*
Notes : init the pcan.
Input : name of usb-pcan which you can get it use commands "ls /dev/pcan*".the default name:"/dev/pcan0".
Output: none.
*/
bhand::bhand(string dev_name)
{
    can_handle.pcan_init(dev_name);
    pcan_handle=can_handle.pcan_handle;
}

bhand::~bhand()
{
    setJointVelocity(FINGER1,0);
    setJointVelocity(FINGER2,0);
    setJointVelocity(FINGER3,0);
    setJointVelocity(SPREAD,0);
}

/*
Notes : send a series command to init the Barett hand using usb-pcan.
      : Warning: it want wait Barett hand for ready(led is green.)
Input : none.
Output: none.
*/
void bhand::barett_hand_init()
{
    TPCANMsg test_data;
    //query the bhand state.
    for(char i=0;i<=3;i++)
    {
        test_data.ID=0xcb+i;
        test_data.LEN=1;
        test_data.MSGTYPE=MSGTYPE_STANDARD;
        test_data.DATA[0]=0x05;
        can_handle.funCAN_Write(pcan_handle,&test_data);
    }

    for(char i=0;i<=3;i++)
    {
        test_data.ID=0xcb+i;
        test_data.LEN=6;
        test_data.MSGTYPE=MSGTYPE_STANDARD;
        test_data.DATA[0]=0xb2;
        for(char j=1;j<6;j++)test_data.DATA[j]=0;
        can_handle.funCAN_Write(pcan_handle,&test_data);
    }

    for(char i=0;i<=3;i++)
    {
        test_data.ID=0xcb+i;
        test_data.LEN=6;
        test_data.MSGTYPE=MSGTYPE_STANDARD;
        for(char j=0;j<6;j++)test_data.DATA[j]=0;
        test_data.DATA[0]=0xce;
        test_data.DATA[2]=0xfa;
        can_handle.funCAN_Write(pcan_handle,&test_data);
    }

    //query the bhand state.
    for(char i=0;i<=3;i++)
    {
        test_data.ID=0xcb+i;
        test_data.LEN=1;
        test_data.MSGTYPE=MSGTYPE_STANDARD;
        test_data.DATA[0]=0x00;
        can_handle.funCAN_Write(pcan_handle,&test_data);
    }
    //query the bhand state.
    for(char i=0;i<=3;i++)
    {
        test_data.ID=0xcb+i;
        test_data.LEN=1;
        test_data.MSGTYPE=MSGTYPE_STANDARD;
        test_data.DATA[0]=0x5a;
        can_handle.funCAN_Write(pcan_handle,&test_data);
    }
    //query the bhand state.
    for(char i=0;i<=3;i++)
    {
        test_data.ID=0xcb+i;
        test_data.LEN=1;
        test_data.MSGTYPE=MSGTYPE_STANDARD;
        test_data.DATA[0]=0x08;
        can_handle.funCAN_Write(pcan_handle,&test_data);
    }
    //init the bhand.
    for(char i=0;i<=3;i++)
    {
        test_data.ID=0xcb+i;
        test_data.LEN=4;
        test_data.MSGTYPE=MSGTYPE_STANDARD;
        test_data.DATA[0]=0x9d;
        test_data.DATA[1]=0x00;
        test_data.DATA[2]=0x0d;
        test_data.DATA[3]=0x00;
        can_handle.funCAN_Write(pcan_handle,&test_data);
    }
    sleep(5);
    printf("BHand Init successfully!\n");
}

/*
Notes : set the Barett hand joint in a input position.
        FINGER1,FINGER2,FINGER3 the max value Position in float:2.45
        SPREAD the max value Position in float:0.45
Input : the position of finger 1, finger 2, finger 3, and base in float format.
Output: none.
*/
void bhand::setJointPosition(float f1_pos_f, float f2_pos_f, float f3_pos_f, float sp_pos_f)
{
    TPCANMsg CANMsg;
    long f1_pos,f2_pos,f3_pos,sp_pos;
    f1_pos=rad_to_enc(f1_pos_f);
    f2_pos=rad_to_enc(f2_pos_f);
    f3_pos=rad_to_enc(f3_pos_f);
    sp_pos=rad_to_enc(sp_pos_f);

    CANMsg.LEN = 6;
    CANMsg.MSGTYPE =MSGTYPE_STANDARD;

    CANMsg.DATA[0] = 0xba;
    CANMsg.DATA[1] = 0;
    CANMsg.DATA[2] = 0;
    CANMsg.DATA[3] = 0;
    CANMsg.DATA[4] = 0;
    CANMsg.DATA[5] = 0;

    CANMsg.DATA[2] =  f1_pos & 0xff;
    CANMsg.DATA[3] = (f1_pos>>8) & 0xff;
    CANMsg.DATA[4] = (f1_pos>>16) & 0xff;

    CANMsg.ID = 0xcb;
    can_handle.funCAN_Write(pcan_handle,&CANMsg);
    usleep(1000);
    CANMsg.DATA[2] = f2_pos & 0xff;
    CANMsg.DATA[3] = (f2_pos>>8) & 0xff;
    CANMsg.DATA[4] = (f2_pos>>16) & 0xff;

    CANMsg.ID = 0xcc;
    can_handle.funCAN_Write(pcan_handle,&CANMsg);
    usleep(1000);

    CANMsg.DATA[2] = f3_pos & 0xff;
    CANMsg.DATA[3] = (f3_pos>>8) & 0xff;
    CANMsg.DATA[4] = (f3_pos>>16) & 0xff;

    CANMsg.ID = 0xcd;
    can_handle.funCAN_Write(pcan_handle,&CANMsg);
    usleep(1000);

    CANMsg.DATA[2] = sp_pos & 0xff;
    CANMsg.DATA[3] = (sp_pos>>8) & 0xff;
    CANMsg.DATA[4] = (sp_pos>>16) & 0xff;

    CANMsg.ID = 0xce;
    can_handle.funCAN_Write(pcan_handle,&CANMsg);
    usleep(1000);
}

/*
Notes : set a barett hand joint in a radian speed in int format.
        include mode switch, and control enable.
Input : puckID is FINGER1,FINGER2,FINGER3 or SPREAD,velocity is better to get using function rad_to_enc().
Output: none.
*/
void bhand::set_velocity(int puckID, int velocity)
{
    //First set TSTOP to 0.
    set_property(puckID, TSTOP, 0);
    //Set Velocity
    set_property(puckID, V, velocity);
    //Set mode to allow the puck to move.
    set_property(puckID, MODE, MODE_VEL);
}
/*
Notes : set a barett hand joint in a radian speed in float format.
Input : finger is oneof the FINGER1,FINGER2,FINGER3 or SPREAD, rad_s is the radian speed in float format.
Output: none.
*/
void bhand::setJointVelocity(int finger, float rad_s)
{
    short int value =rad_to_enc(rad_s, BASE_TYPE) / 1000;
    set_velocity(finger,value);
}

/*
Notes : wait the current action of barett hand finished. and if wait time exceeds 10s, it will return.
Input : none.
Output: none.
Warnning: the ros::Time::now() function isn't contain in standard c++ lib, if you want use this lib in a stardard c++ environment, please replace it.
*/
bool bhand::wait_act_finish()
{
    double start_sec,delta_time;
    start_sec=ros::Time::now().toSec();
    while(fabs(get_velocity(FINGER1))>=0.001 || fabs(get_velocity(FINGER2))>=0.001 || fabs(get_velocity(FINGER3))>=0.001 || fabs(get_velocity(SPREAD))>=0.001)
    {
        delta_time=ros::Time::now().toSec()-start_sec;
        if(fabs(delta_time)>=10.0)
        {
            return false;
        }
    }
    printf("Act Delta time:%f\n",delta_time);
    return true;
}

/*
 Notes : get current positon of bhand joint.
         FINGER1,FINGER2,FINGER3 the max value Position in float:2.45
         SPREAD the max value Position in float:0.45
 Input : msgID is oneof the FINGER1,FINGER2,FINGER3 and SPREAD, the default value of depth is 1.
Output : a unsigned int number which represent the value of joint motor encoder.
Example: xxx.get_position(FINGER1)
*/
uint32_t bhand::get_position(int msgID, int depth)
{
    TPCANRdMsg read_result;
    uint32_t value;
//    ROS_INFO("Depth:%d",depth);
    if(depth != 0)
    {
        unsigned int data[]={0x30};
        send_msg(msgID+0xc0,data,sizeof(data)/sizeof(data[0]),9000);
    }
    if (can_handle.funLINUX_CAN_Read(pcan_handle, &read_result)) {
        //perror("receivetest: LINUX_CAN_Read()");
    }
    int received_puck=(read_result.Msg.ID-0x146)/32+10;
    if(received_puck == msgID)
    {
        unsigned int data[read_result.Msg.LEN];
        for(int i=0;i<read_result.Msg.LEN;i++)
            data[i]= read_result.Msg.DATA[i];
        if(read_result.Msg.LEN==6)
            value = (0x1000000 * data[5]) + (0x0010000 * data[4]) + (0x0000100 * data[3]) + (0x0000001 * data[2]);
        return value;
    }

    if(depth<=20)    return get_position(msgID,depth+1);
    else return 0;
}

/*
 Notes : get current joint velocity in float format.
         msgID is oneof the the FINGER1,FINGER2,FINGER3 and SPREAD.
 Input : msgID is the same as the puckID which is set in prior variable definition.
Output : the value of joint velocity in float format.
Example: xxx.get_velocity(FINGER1).
Warnning: the ros::Time::now() function isn't contain in standard c++ lib, if you want use this lib in a stardard c++ environment, please replace it.
*/
float bhand::get_velocity(int msgID)
{
    float start_pos,end_pos,delta_pos;
    double start_sec,end_sec,delta_time;
    float velocity;
    start_pos=enc_to_rad(get_position(msgID));
    start_sec=ros::Time::now().toSec();

    end_pos=enc_to_rad(get_position(msgID));
    end_sec=ros::Time::now().toSec();

    delta_pos   = end_pos-start_pos;
    delta_time  = end_sec-start_sec;

    velocity=delta_pos/delta_time;
    //printf("Delta Time:%f,Position:%f,Velocity:%f\n",delta_time,delta_pos,velocity);
    return velocity;
}

/*
Notes : an auxiliary function, Set property to a given value.
Input : The id of the finger to initialize.
        param msgID: The puck or group whose property will be set.
        param propID: The number corresponding to the property to be set.
        param value: The value to which the property will be set.
Output: none.
*/
void bhand::set_property(int msgID, int propID, int value)
{
    int is32bits[12] = {48, 50, 52, 54, 56, 58, 66, 68, 74, 88, 96, 98};
    bool is32bits_flag=false;
    for(int i=0;i<12;i++)
    {
        if(propID==is32bits[i])
        {
            is32bits_flag=true;
            break;
        }
    }
    if (is32bits_flag)
        set_32(msgID, propID, value);
    else
        set_16(msgID, propID, value);
}

/*
Notes : Set property to a given value for a 16 bit property.
        Avoid usage of this method. Use self.set_property instead.
Input : param msgID: The puck or group whose property will be set.
        param propID: The number corresponding to the property to be set.
        param value: The value to which the property will be set.
Output: none.
*/
void bhand::set_16(int msgID, int propID, int tmp_value)
{
    unsigned int value;
    value=tmp_value;
    //short int data[]={0x80+propID, 0, value&0xff, (value>>8) & 0xff};
    unsigned int data[]={(unsigned int)(0x80+propID), 0, (unsigned int)(value%256),(unsigned int)(value/256)};
    send_msg(msgID,data,sizeof(data)/sizeof(data[0]));
}

/*
Notes : Set property to a given value for a 32 bit property.
        Avoid usage of this method. Use self.set_property instead.
Input : param msgID: The puck or group whose property will be set.
        param propID: The number corresponding to the property to be set.
        param value: The value to which the property will be set.
Output: none.
*/
void bhand::set_32(int msgID, int propID, int tmp_value)
{
    unsigned int value;
    value=tmp_value;
    unsigned int data[]= {(unsigned int)(0x80+propID), 0, (unsigned int)(value%0x100), (unsigned int)(value/0x100)%0x100, (unsigned int)(value/0x10000)%0x100,(unsigned int)(value/0x1000000)};
    send_msg(msgID,data,sizeof(data)/sizeof(data[0]));
}


/*
Notes : Given the readian measure of an angle, return it in encoder counts.
Input : param rad: Radians
        param type: represent the different joint, such as BASE_TYPE, TIP_TYPE, SPREAD_TYPE. the default value is BASE_TYPE.
Output: Encoder counts.
*/
int bhand::rad_to_enc(float rad, int type)
{
    int motion_limit,tics,enc;
    motion_limit = BASE_LIMIT;
    tics = MAX_ENCODER_TICKS;

    if( type == TIP_TYPE)
    {
        motion_limit = TIP_LIMIT;
        tics = MAX_FINGERTIP_TICKS;
    }
    else if (type == SPREAD_TYPE)
    {
        motion_limit = SPREAD_LIMIT;
        tics = MAX_SPREAD_TICKS;
    }

    float PI = 3.141592653589;
    enc =(int)(rad / ((motion_limit*PI/180)/tics));
    return enc;
}

/*
Notes : Given an angle in encoder counts, return the radian measure of the angle that represents.
Input : param rad: encoder counts
        param type: represent the different joint, such as BASE_TYPE, TIP_TYPE, SPREAD_TYPE. the default value is BASE_TYPE.
Output: Radians.
*/
float bhand::enc_to_rad(uint32_t enc, int type)
{
    int motion_limit,tics;
    float rad;
    motion_limit = BASE_LIMIT;
    tics = MAX_ENCODER_TICKS;

    if( type == TIP_TYPE)
    {
        motion_limit = TIP_LIMIT;
        tics = MAX_FINGERTIP_TICKS;
    }
    else if (type == SPREAD_TYPE)
    {
        motion_limit = SPREAD_LIMIT;
        tics = MAX_SPREAD_TICKS;
    }


    float PI = 3.141592653589;
    rad = enc * (motion_limit*PI/180)/tics;
    return rad;
}

/*
 Notes : Send a general message to PCAN_USBBUS0. This can be a get, set, or even garbage.
         It does not apply any sleep
 Input : param msgID: The puck or group to which the message will be sent.
         param data: The array containing the data for the TPCANMsg.
         type data: Array[]
Output : the return status after writing in the bus
Example: unsigned int data[]={0x36};
         send_msg(force_id,data,sizeof(data)/sizeof(data[0]),9000);
*/
DWORD bhand::send_msg(int msgID, unsigned int data[], int len, unsigned int delay_us)
{
    TPCANMsg msg;
    msg.LEN =len;
    msg.MSGTYPE =MSGTYPE_STANDARD;
    for(int i=0;i<len;i++)
    {
        msg.DATA[i]=data[i];
    }
    msg.ID = msgID;
    DWORD stat=can_handle.funCAN_Write(pcan_handle,&msg);
    check_error(stat,"write");
    //print_message(&msg);
    usleep(delay_us);
    return stat;
}

/*
Notes : Checks error on the CAN Bus.
Input : param result: The number corresponding to the error returned.
        param location_of_error: A description of where the error occurred.
        type location_of_error: str
Output: none.
*/
void bhand::check_error(__u32 result, string location_of_error)
{
    if(result==CAN_ERR_OK)
        return;
    else
    {
        printf("CAN Error Number:%x, while attempting to %s ",result,location_of_error.c_str());
    }
}

/*
Notes : init the 6 axis force sensor, the process is given by official c++ program.
Input : none.
Output: none.
*/
void bhand::axisForce_init()
{
    //wake puck
    wavePuck(0,8);
    //tare
    setPropertySlow(0,FTSENSORID,FTDATAPROPERTY,0,0);
    axisSensor.force_x=0;
    axisSensor.force_y=0;
    axisSensor.force_z=0;
    axisSensor.torque_x=0;
    axisSensor.torque_y=0;
    axisSensor.torque_z=0;
    sleep(2);
    printf("6 axis force sensor init successfully!\n");
}

/*
Notes : enable the 6 axis force sensor.
Input : param id: can message id.
Output: the status after can write.
*/
int bhand::wavePuck(int bus, int id)
{
    TPCANMsg msgOut;
    DWORD err;

    // Generate the outbound message
    msgOut.ID = id;
    msgOut.MSGTYPE = MSGTYPE_STANDARD;
    msgOut.LEN = 4;
    msgOut.DATA[0] = 0x85; // Set Status
    msgOut.DATA[1] = 0x00;
    msgOut.DATA[2] = 0x02; // Status = Ready
    msgOut.DATA[3] = 0x00;

    // Send the message
    err=can_handle.funCAN_Write(pcan_handle,&msgOut);
    sleep(1);

    return(err);
}

/*
Notes : used to set and get prop.
Input : property: The property being compiled (use the enumerations in btcan.h)
        longVal: The value to set the property to
        *data: A pointer to a character buffer in which to build the data payload
        *dataLen: A pointer to the total length of the data payload for this packet
        isForSafety: A flag indicating whether this packet is destined for the safety circuit or not
Output: the status after can write.
*/
int bhand::compile(int property, long longVal, unsigned char *data, int *dataLen, int isForSafety)
{
    int i;

    /* Insert the property */
    data[0] = property;
    data[1] = 0; /* To align the values for the tater's DSP */

    /* Append the value */
    for (i = 2; i < 6; i++)
    {
       data[i] = (char)(longVal & 0x000000FF);
       longVal >>= 8;
    }

    /* Record the proper data length */
    *dataLen = 6; //(dataType[property] & 0x0007) + 2;

    return (0);
}

/*
Notes : it is not clear. I porting it from the offical monitor cpp named MonitorForceTorque.cpp.
Input : ...
Output: ...
*/
int bhand::setPropertySlow(int bus, int id, int property, int verify, long value)
{
    TPCANMsg msgOut, msgIn;
    DWORD err;
    int dataHeader, i;
    long response;
    //unsigned char   data[8];
    int             len;

    //syslog(LOG_ERR, "About to compile setProperty, property = %d", property);
    // Compile 'set' packet
    err = compile(property, value, msgOut.DATA, &len, 0);

    // Generate the outbound message
    msgOut.ID = id;
    msgOut.MSGTYPE = MSGTYPE_STANDARD;
    msgOut.LEN = len;

    //syslog(LOG_ERR, "After compilation data[0] = %d", data[0]);
    msgOut.DATA[0] |= 0x80; // Set the 'Set' bit

    // Send the message
    can_handle.funCAN_Write(pcan_handle,&msgOut);

    // BUG: This will not verify properties from groups of pucks
    return(0);
}

/*
 Notes : get force data of axis force sensor in a iteration query method.
 Input : force_id: the id of query message. depth represent the iteration depth.
 Output: the axis force of X,Y,Z in a constructs of axisSensorData format.
Example: xxx.get_AxisForce (AxisForceQuery)
*/
axisSensorData bhand::get_AxisForce(int force_id,int depth)
{
    TPCANRdMsg read_result;
    short int x_force,y_force,z_force;
    if(depth != 0)
    {
        unsigned int data[]={0x36};
        send_msg(force_id,data,sizeof(data)/sizeof(data[0]),9000);
    }
    if (can_handle.funLINUX_CAN_Read(pcan_handle, &read_result)) {
        //perror("receivetest: LINUX_CAN_Read()");
    }
    int received_puck=read_result.Msg.ID;
    if(received_puck == AxisForce)
    {
        if(read_result.Msg.LEN==6)
        {
            x_force=read_result.Msg.DATA[1];
            x_force<<=8;
            x_force|=read_result.Msg.DATA[0];

            y_force=read_result.Msg.DATA[3];
            y_force<<=8;
            y_force|=read_result.Msg.DATA[2];

            z_force=read_result.Msg.DATA[5];
            z_force<<=8;
            z_force|=read_result.Msg.DATA[4];
            axisSensor.force_x=x_force/256.0;
            axisSensor.force_y=y_force/256.0;
            axisSensor.force_z=z_force/256.0;
            return axisSensor;
        }
    }

    if(depth<=20)
    {
        usleep(9000);
        return get_AxisForce(force_id,depth+1);
    }
    else
    {
        printf("get AxisForce FAILED\n");
        return axisSensor;
    }
}

/*
 Notes : get torque data of axis force sensor in a iteration query method.
 Input : torque_id: the id of query message. depth represent the iteration depth.
 Output: the axis torque of X,Y,Z in a constructs of axisSensorData format.
Example: xxx.get_AxisTorque(AxisTorqueQuery)
*/
axisSensorData bhand::get_AxisTorque(int torque_id,int depth)
{
    TPCANRdMsg read_result;
    short int torque_x,torque_y,torque_z;
    if(depth != 0)
    {
        unsigned int data[]={0x36};
        send_msg(torque_id,data,sizeof(data)/sizeof(data[0]),9000);
    }
    if (can_handle.funLINUX_CAN_Read(pcan_handle, &read_result)) {
        //perror("receivetest: LINUX_CAN_Read()");
    }
    int received_puck=read_result.Msg.ID;


    if(received_puck == AxisTorque)
    {
        if(read_result.Msg.LEN==6)
        {
            torque_x=read_result.Msg.DATA[1];
            torque_x<<=8;
            torque_x|=read_result.Msg.DATA[0];

            torque_y=read_result.Msg.DATA[3];
            torque_y<<=8;
            torque_y|=read_result.Msg.DATA[2];

            torque_z=read_result.Msg.DATA[5];
            torque_z<<=8;
            torque_z|=read_result.Msg.DATA[4];
            axisSensor.torque_x=torque_x/256.0;
            axisSensor.torque_y=torque_y/256.0;
            axisSensor.torque_z=torque_z/256.0;
            return axisSensor;
        }
    }

    if(depth<=20)
    {
        usleep(9000);
        return get_AxisTorque(torque_id,depth+1);
    }
    else
    {
        printf("get AxisTorque FAILED\n");
        return axisSensor;
    }

}

/*
Notes : display the constructures data of axisSensorData.
Input : the axisSensorData
Output: none.
*/
void bhand::displayAxisSensordata(axisSensorData axisData)
{
    printf("Axis  Force x:%f,y:%f,z:%f\n",axisData.force_x,axisData.force_y,axisData.force_z);
    printf("Axis Torque x:%f,y:%f,z:%f\n",axisData.torque_x,axisData.torque_y,axisData.torque_z);
}

/*
Notes : display the force data of axisSensorData.
Input : the axisSensorData
Output: none.
*/
void bhand::displayAxisForceData(axisSensorData axisData)
{
    printf("Axis  Force x:%f,y:%f,z:%f\n",axisData.force_x,axisData.force_y,axisData.force_z);
}

/*
Notes : display the torque data of axisSensorData.
Input : the axisSensorData
Output: none.
*/
void bhand::displayAxisTorqueData(axisSensorData axisData)
{
    printf("Axis Torque x:%f,y:%f,z:%f\n",axisData.torque_x,axisData.torque_y,axisData.torque_z);
}
