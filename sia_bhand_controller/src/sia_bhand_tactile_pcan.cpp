#include <ros/ros.h>
#include <stdio.h>
#include <dlfcn.h>
#include <libpcan.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include "sia_bhand_controller/ForceData.h"
//Header file that process communication need
#include <unistd.h>
#include <stdlib.h>
//#include <stdio.h>
#include <sys/stat.h>
//#include <fcntl.h>
#include "string"
/*
 *
Author  :Hekai SIA
Date    :2016,10,12
Usage   :1,acquire the message on can bus,and publish the id and data of the message as a ros
        :topic.
        :2,send sensor data to UI software using for display by inter-process of pipe fifo.
E-mail  :hekai@sia.cn;susthekai@qq.com
Debug note:
1,the can_test process can not be ended when press ctrl+c, and the do_exit() can not be run.
2,when run the command-"rostopic echo /ArrayForceDEV",there are some negtive value even if i
added abs function when i copy the data from can message.
result:i use the int8 errorly, and the riginal data was unsigned char,i correct it as int16.
3,added the inter-process communication using Pipe Fifo, and it send 64B one time.
WARNING:Please resect the author the work,Don't use it as business application.
*
*/
using namespace std;

//using for Pipe inter-process communication create.....START
#define SERVER_FIFO_NAME "/tmp/serv_fifo_ui"
#define CLIENT_FIFO_NAME "/tmp/client_fifo_ui"

#define BUFFER_SIZE 8

struct message
{
    pid_t client_pid;
    int data[BUFFER_SIZE][BUFFER_SIZE];
};

struct sensor_flag
{
    char row0_flag,row1_flag,row2_flag,row3_flag,row4_flag,row5_flag,row6_flag,row7_flag;
};
sensor_flag sensor1_flag={0},sensor2_flag={0},sensor3_flag={0};

int force_fifo_data[BUFFER_SIZE][BUFFER_SIZE]={0};
unsigned int send_count=0;
//using for Pipe inter-process communication create......END

//define mapping function according to target function in libpcan.h
typedef DWORD   (*funCAN_Init_TYPE)(HANDLE hHandle, WORD wBTR0BTR1, int nCANMsgType);
typedef HANDLE  (*funLINUX_CAN_Open_TYPE)(const char *szDeviceName, int nFlag);
typedef DWORD   (*funCAN_VersionInfo_TYPE)(HANDLE hHandle, LPSTR lpszTextBuff);
typedef DWORD   (*funCAN_Close_TYPE)(HANDLE hHandle);
typedef DWORD   (*funLINUX_CAN_Read_TYPE)(HANDLE hHandle, TPCANRdMsg* pMsgBuff);
typedef DWORD   (*funCAN_Status_TYPE)(HANDLE hHandle);
typedef int     (*funnGetLastError_TYPE)(void);
typedef DWORD   (*funCAN_Write_TYPE)(HANDLE hHandle, TPCANMsg* pMsgBuff);

//the target device name
#define DEFAULT_NODE        "/dev/pcan0"
//tactile sensor ID
#define No1_TactileSensorID 0x06e0//1760
#define No2_TactileSensorID 0X0708//1800,board:003
#define No3_TactileSensorID 0X0758//1880
//GLOBALS
void *libm_handle = NULL;//define pointer used for file acess of libpcan.so
HANDLE pcan_handle =NULL;//void *pcan_handle


sia_bhand_controller::ForceData force_msg;

//function declaration
int     read_loop(HANDLE h,ros::Publisher pub,int server_fifo_fd, bool display_on,bool publish_on,bool fifo_send_on);
void    print_message(TPCANMsg *m);
void    print_message_ex(TPCANRdMsg *mr);
void    do_exit(void *file,HANDLE h,int error);
void    signal_handler(int signal);
void    init();
void    publish_forcedata(TPCANRdMsg *mr,ros::Publisher force_pub,int server_fifo_fd,bool fifo_send_on);

//define function pointer,there is a one-to-one mapping between target function and your defined function
funCAN_Init_TYPE        funCAN_Init;
funLINUX_CAN_Open_TYPE  funLINUX_CAN_Open;
funCAN_VersionInfo_TYPE funCAN_VersionInfo;
funCAN_Close_TYPE       funCAN_Close;
funLINUX_CAN_Read_TYPE  funLINUX_CAN_Read;
funCAN_Status_TYPE      funCAN_Status;
funnGetLastError_TYPE   funnGetLastError;
funCAN_Write_TYPE       funCAN_Write;

//Pipe FIFO:inter-process communication,function declarations
int pipe_fifo_init(const char  *server_pipename);
void pipe_fifo_datasend(int server_fifo_fd, message msg,char display);
void Fifo_array_datasend(sia_bhand_controller::ForceData force_msg,int server_fifo_fd,int sensor_id,sensor_flag *sensorflag);


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"PCAN_Test");
    init();

    //parameter load test start ......

    string dev_name;
    ros::param::get("~dev_name",dev_name);
    ROS_INFO("PCAN Device:%s",dev_name.c_str());

    bool fifo_send_on=false;
    ros::param::get("~fifo_send_on",fifo_send_on);
    ROS_INFO("FIFO send :%s",fifo_send_on==false?"false":"true");

    bool display_on=false;
    ros::param::get("~display_on",display_on);
    ROS_INFO("Data display:%s",display_on==false?"false":"true");
    //parameter load test end ......

    //using for Pipe inter-process communication create.....START
    int server_fifo_fd;
    message msg;

    if(fifo_send_on) server_fifo_fd=pipe_fifo_init(SERVER_FIFO_NAME);

    //using for Pipe inter-process communication create......END

    //load libpcan.so using dlopen function,return handle for further use
    libm_handle = dlopen("libpcan.so", RTLD_LAZY );
    if (!libm_handle){
        printf("Open Error:%s.\n",dlerror());//if file can't be loaded,return null,get reason using dlerror function
        return 0;
    }

    char *errorInfo;//error information pointer
    //one-to-one mapping using dlsym function,if return null,mapping would be failed
    funCAN_Init         =(funCAN_Init_TYPE)         dlsym(libm_handle,"CAN_Init");
    funLINUX_CAN_Open   =(funLINUX_CAN_Open_TYPE)   dlsym(libm_handle,"LINUX_CAN_Open");
    funCAN_Close        =(funCAN_Close_TYPE)        dlsym(libm_handle,"CAN_Close");
    funCAN_VersionInfo  =(funCAN_VersionInfo_TYPE)  dlsym(libm_handle,"CAN_VersionInfo");
    funLINUX_CAN_Read   =(funLINUX_CAN_Read_TYPE)   dlsym(libm_handle,"LINUX_CAN_Read");
    funCAN_Status       =(funCAN_Status_TYPE)       dlsym(libm_handle,"CAN_Status");
    funnGetLastError    =(funnGetLastError_TYPE)    dlsym(libm_handle,"nGetLastError");
    funCAN_Write        =(funCAN_Write_TYPE)        dlsym(libm_handle,"CAN_Write");

    errorInfo = dlerror();//get error using dlerror function,and clear the error list in memory
    if (errorInfo != NULL){
        printf("Dlsym Error:%s.\n",errorInfo);
        return 0;
    }

    char txt[VERSIONSTRING_LEN];            //store information of can version
    unsigned short wBTR0BTR1 = CAN_BAUD_1M; //set the communicate baud rate of can bus
    int nExtended = CAN_INIT_TYPE_ST;       //set can message int standard model
    const char  *szDevNode = DEFAULT_NODE;  //define const pointer point to device name

    if(dev_name.c_str() !="")
        pcan_handle = funLINUX_CAN_Open(dev_name.c_str(), O_RDWR );
    else
    {
        pcan_handle = funLINUX_CAN_Open(szDevNode, O_RDWR );//use mapping function
        dev_name=DEFAULT_NODE;
    }
    //judge whether the call is success.if pcan_handle=null,the call would be failed
    if(pcan_handle){
        printf("CAN Bus test: %s have been opened\n", dev_name.c_str());
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
                    printf("Device Info: %s; CAN_BAUD_1M; CAN_INIT_TYPE_ST\n", dev_name.c_str());
            }
    }
    else
        printf("CAN Bus test: can't open %s\n", dev_name.c_str());

    //initial a talker to publish the force data and can id.
    ros::NodeHandle force_handle;
    ros::Publisher force_pub=force_handle.advertise<sia_bhand_controller::ForceData>("ArrayForcePUB",1);//advertise a topic named "ArrayForceDEV"
    ros::Rate loop_rate(500);
    //data receive test
    while(ros::ok())
    {   //HANDLE h,ros::Publisher pub,int server_fifo_fd,bool display_on,bool publish_on,bool fifo_send_on
        read_loop(pcan_handle,force_pub,server_fifo_fd,display_on,true,fifo_send_on);
        if(fifo_send_on)  printf("Send Count No.%d\t\n",send_count);
        ros::spinOnce();
        loop_rate.sleep();
    }
    close(server_fifo_fd);
    return 0;
}


//read from CAN forever - until manual break
int read_loop(HANDLE h,ros::Publisher pub,int server_fifo_fd, bool display_on,bool publish_on,bool fifo_send_on)
{
    TPCANRdMsg m;
    __u32 status;

    if (funLINUX_CAN_Read(h, &m)) {
        perror("receivetest: LINUX_CAN_Read()");
        return errno;
    }

    if (display_on)
        print_message_ex(&m);
    if (publish_on)
        publish_forcedata(&m,pub,server_fifo_fd,fifo_send_on);

    // check if a CAN status is pending
    if (m.Msg.MSGTYPE & MSGTYPE_STATUS) {
        status = funCAN_Status(h);
        if ((int)status < 0) {
            errno = funnGetLastError();
            perror("receivetest: CAN_Status()");
            return errno;
        }

        printf("receivetest: pending CAN status 0x%04x read.\n",
               (__u16)status);
    }
    return 0;
}

void publish_forcedata(TPCANRdMsg *mr,ros::Publisher force_pub,int server_fifo_fd,bool fifo_send_on)
{
    char sensorflag1=0,sensorflag2=0,sensorflag3=0;

    if((mr->Msg.ID==No1_TactileSensorID+0)||(mr->Msg.ID==No1_TactileSensorID+1)||(mr->Msg.ID==No1_TactileSensorID+2)||(mr->Msg.ID==No1_TactileSensorID+3)||(mr->Msg.ID==No1_TactileSensorID+4)||(mr->Msg.ID==No1_TactileSensorID+5)||(mr->Msg.ID==No1_TactileSensorID+6)||(mr->Msg.ID==No1_TactileSensorID+7))
    sensorflag1=1;
    if((mr->Msg.ID==No2_TactileSensorID+0)||(mr->Msg.ID==No2_TactileSensorID+1)||(mr->Msg.ID==No2_TactileSensorID+2)||(mr->Msg.ID==No2_TactileSensorID+3)||(mr->Msg.ID==No2_TactileSensorID+4)||(mr->Msg.ID==No2_TactileSensorID+5)||(mr->Msg.ID==No2_TactileSensorID+6)||(mr->Msg.ID==No2_TactileSensorID+7))
    sensorflag2=1;
    if((mr->Msg.ID==No3_TactileSensorID+0)||(mr->Msg.ID==No3_TactileSensorID+1)||(mr->Msg.ID==No3_TactileSensorID+2)||(mr->Msg.ID==No3_TactileSensorID+3)||(mr->Msg.ID==No3_TactileSensorID+4)||(mr->Msg.ID==No3_TactileSensorID+5)||(mr->Msg.ID==No3_TactileSensorID+6)||(mr->Msg.ID==No3_TactileSensorID+7))
    sensorflag3=1;

    if(sensorflag1||sensorflag2||sensorflag3)
    {
        force_msg.id=mr->Msg.ID;
        for(__u8 i=0;i<8;i++)
            force_msg.data[i]=mr->Msg.DATA[i];
        force_pub.publish(force_msg);
        //fifo data fill, inter-process communication
//        msg.client_pid=force_msg.id;
//        for(char i=0;i<BUFFER_SIZE;i++) msg.data[i]=force_msg.data[i];
        if(fifo_send_on)
        {
            Fifo_array_datasend(force_msg,server_fifo_fd,No1_TactileSensorID,&sensor1_flag);
            Fifo_array_datasend(force_msg,server_fifo_fd,No2_TactileSensorID,&sensor2_flag);
            Fifo_array_datasend(force_msg,server_fifo_fd,No3_TactileSensorID,&sensor3_flag);
        }
    }

    sensorflag1=0;sensorflag2=0;sensorflag3=0;
}


void print_message(TPCANMsg *m)
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

void print_message_ex(TPCANRdMsg *mr)
{
    printf("%u.%3u ", mr->dwTime, mr->wUsec);
    print_message(&mr->Msg);
}

// exit handler
void do_exit(void *file,HANDLE h,int error)
{
    //Must close h handle firstly,then close file using dlclose
    if (h) {
        funCAN_Close(h);
    }
    printf("\nCAN Bus test: finished (%d).\n\n", error);
    //after call the target function in ELF object,close it using dlclose
    dlclose(file);
    exit(error);
}

// the signal handler for manual break Ctrl-C
void signal_handler(int signal)
{
    do_exit(libm_handle,pcan_handle,0);
}

// what has to be done at program start
void init()
{
    /* install signal handlers */
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
}

//Pipe FIFO:inter-process communication,function defination
int pipe_fifo_init(const char  *server_pipename)
{
    int res;
    if(access(server_pipename,F_OK) == -1)
    {
        //管道文件不存在
        //创建命名管道
        res = mkfifo(server_pipename, 0777);
        if(res != 0)
        {
            printf("Could not create fifo %s\n", server_pipename);
            exit(EXIT_FAILURE);
        }
    }

    printf("Process %d opening FIFO\n", getpid());
    //以读写阻塞方式打开FIFO文件
    int server_fifo_fd = open(server_pipename, O_RDWR);
    return server_fifo_fd;


}
void pipe_fifo_datasend(int server_fifo_fd, message msg,char display)
{
    int res;
//    if(display==1)
//    {
//        printf("PIPE %d sent:\n", msg.client_pid);//display the sending data
//        for(int i=0;i<BUFFER_SIZE;i++)
//        {
//            printf("%3d",msg.data[i]);
//            if(i==BUFFER_SIZE-1)printf("\n");
//        }
//    }
    if(display==1)
    {
        printf("ID:%d,Data:\n",msg.client_pid);
        for(char j=0;j<BUFFER_SIZE;j++)
        {
            for(char i=0;i<BUFFER_SIZE;i++)
            {
                printf("%4d",msg.data[j][i]);
                if(i==BUFFER_SIZE-1)printf("\n");
            }
        }
    }

    res= write(server_fifo_fd, &msg, sizeof(msg));
    if(res<0)printf("\nERROR in Pipe Fifo write %d\n",res);
}


void Fifo_array_datasend(sia_bhand_controller::ForceData force_msg,int server_fifo_fd,int sensor_id,sensor_flag *sensorflag)
{
    if(force_msg.id==sensor_id+0 )
    {
        for(char i=0;i<8;i++)
            force_fifo_data[0][i]=force_msg.data[i];
        sensorflag->row0_flag=1;
    }
    if(force_msg.id==sensor_id+1 )
    {
        for(char i=0;i<8;i++)
            force_fifo_data[1][i]=force_msg.data[i];
        sensorflag->row1_flag=1;
    }
    if(force_msg.id==sensor_id+2 )
    {
        for(char i=0;i<8;i++)
            force_fifo_data[2][i]=force_msg.data[i];
        sensorflag->row2_flag=1;
    }
    if(force_msg.id==sensor_id+3 )
    {
        for(char i=0;i<8;i++)
            force_fifo_data[3][i]=force_msg.data[i];
        sensorflag->row3_flag=1;
    }
    if(force_msg.id==sensor_id+4 )
    {
        for(char i=0;i<8;i++)
            force_fifo_data[4][i]=force_msg.data[i];
        sensorflag->row4_flag=1;
    }
    if(force_msg.id==sensor_id+5 )
    {
        for(char i=0;i<8;i++)
            force_fifo_data[5][i]=force_msg.data[i];
        sensorflag->row5_flag=1;
    }
    if(force_msg.id==sensor_id+6 )
    {
        for(char i=0;i<8;i++)
            force_fifo_data[6][i]=force_msg.data[i];
        sensorflag->row6_flag=1;
    }
    if(force_msg.id==sensor_id+7 )
    {
        for(char i=0;i<8;i++)
            force_fifo_data[7][i]=force_msg.data[i];
        sensorflag->row7_flag=1;
    }
    if(sensorflag->row0_flag && sensorflag->row1_flag && sensorflag->row2_flag && sensorflag->row3_flag && sensorflag->row4_flag && sensorflag->row5_flag && sensorflag->row6_flag && sensorflag->row7_flag)
    {
        send_count++;
        message msg;
        msg.client_pid=sensor_id;
        for(char i=0;i<BUFFER_SIZE;i++)
            for(char j=0;j<BUFFER_SIZE;j++)
            {
                msg.data[i][j]=force_fifo_data[i][j];
            }
        pipe_fifo_datasend(server_fifo_fd, msg,0);
        *(sensorflag)={0};
    }
}

/*
 *
 *
        if(force_msg.id==No1_TactileSensorID+0 )
        {
            for(char i=0;i<8;i++)
            force_fifo_data[0][i]=force_msg.data[i];
            sensor1_count++;
        }
        if(force_msg.id==No1_TactileSensorID+1 )
        {
            for(char i=0;i<8;i++)
            force_fifo_data[1][i]=force_msg.data[i];
            sensor1_count++;
        }
        if(force_msg.id==No1_TactileSensorID+2 )
        {
            for(char i=0;i<8;i++)
            force_fifo_data[2][i]=force_msg.data[i];
            sensor1_count++;
        }
        if(force_msg.id==No1_TactileSensorID+3 )
        {
            for(char i=0;i<8;i++)
            force_fifo_data[3][i]=force_msg.data[i];
            sensor1_count++;
        }
        if(force_msg.id==No1_TactileSensorID+4 )
        {
            for(char i=0;i<8;i++)
            force_fifo_data[4][i]=force_msg.data[i];
            sensor1_count++;
        }
        if(force_msg.id==No1_TactileSensorID+5 )
        {
            for(char i=0;i<8;i++)
            force_fifo_data[5][i]=force_msg.data[i];
            sensor1_count++;
        }
        if(force_msg.id==No1_TactileSensorID+6 )
        {
            for(char i=0;i<8;i++)
            force_fifo_data[6][i]=force_msg.data[i];
            sensor1_count++;
        }
        if(force_msg.id==No1_TactileSensorID+7 )
        {
            for(char i=0;i<8;i++)
            force_fifo_data[7][i]=force_msg.data[i];
            sensor1_count++;
        }
        if(sensor1_count==8)
        {
            sensor1_count=0;
            msg.client_pid=No1_TactileSensorID;
            for(char i=0;i<BUFFER_SIZE;i++)
                for(char j=0;j<BUFFER_SIZE;j++)
                {
                    msg.data[i][j]=force_fifo_data[i][j];
                }
            pipe_fifo_datasend(server_fifo_fd, msg,1);
        }
 *
 *
 */



