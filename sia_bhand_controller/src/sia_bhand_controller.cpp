#include <ros/ros.h>
#include <stdio.h>
#include <dlfcn.h>
#include <libpcan.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include "id_data_msgs/ID_Data.h"//using for notie event
#include "ros/callback_queue.h"
#include "string"
#include "sia_bhand_api.h"

#include "time.h"
using namespace std;

//the target device name
#define DEFAULT_NODE "/dev/pcan0"

#define FTSENSORID (8)
#define FTDATAPROPERTY (54)


//GLOBALS
HANDLE pcan_handle =NULL;//void *pcan_handle
id_data_msgs::ID_Data axis_force_msg;
bool force_display_on=true;
float force_threshold=4.0;
float torque_threshold=10.0;
double force_danger=15.0;
double torque_danger=30.0;
double left_torque_x_threshold=-1.0;
double right_torque_x_threshold=1.0;



//globals
bool close_hand_flag=false;
bool open_hand_flag=false;

class notice_pub_sub
{
public:
    boost::function<void (const id_data_msgs::ID_Data::ConstPtr&)> notice_pub_sub_msgCallbackFun;

    notice_pub_sub();
    void notice_pub_sub_listener();
    void notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data);
    void notice_display(id_data_msgs::ID_Data notice_msg,bool set);
    void notice_sub_spinner(char set);
    void notice_data_clear(id_data_msgs::ID_Data *test);
private:
    ros::NodeHandle notice_handle;
    ros::Subscriber notice_subscriber;
    ros::Publisher notice_publisher;
    ros::SubscribeOptions notice_ops;
    ros::AsyncSpinner *notice_spinner;
    ros::CallbackQueue notice_callbackqueue;
    void notice_msgCallback(const id_data_msgs::ID_Data::ConstPtr &notice_msg);
};

notice_pub_sub::notice_pub_sub()
{
    notice_pub_sub_msgCallbackFun=boost::bind(&notice_pub_sub::notice_msgCallback,this,_1);
    notice_ops=ros::SubscribeOptions::create<id_data_msgs::ID_Data>(
                "/notice",
                1,
                notice_pub_sub_msgCallbackFun,
                ros::VoidPtr(),
                &notice_callbackqueue
                );
    notice_subscriber=notice_handle.subscribe(notice_ops);
    notice_spinner=new ros::AsyncSpinner(1,&notice_callbackqueue);

    notice_publisher=notice_handle.advertise<id_data_msgs::ID_Data>("/notice",1);
}

void notice_pub_sub::notice_pub_sub_pulisher(id_data_msgs::ID_Data id_data)
{
    notice_publisher.publish(id_data);
}

void notice_pub_sub::notice_display(id_data_msgs::ID_Data notice_msg,bool set)
{

    if(set)
    {
        printf("REC Notice message,ID: %d,Data: ",notice_msg.id);
        for(char i=0;i<8;i++)
        {
            printf("%d ",notice_msg.data[i]);            
        }
        printf("\n");

    }

}
void notice_pub_sub::notice_msgCallback(const id_data_msgs::ID_Data::ConstPtr &notice_msg)
{

    id_data_msgs::ID_Data notice_message;
    notice_message.id=0;
    for(char i=0;i<8;i++)notice_message.data[i]=0;

    notice_message.id=notice_msg->id;
    for(char i=0;i<8;i++)notice_message.data[i]=notice_msg->data[i];

    notice_pub_sub::notice_display(notice_message,true);

    if(notice_message.id==1 && notice_message.data[0]==1)//close flag
    {
        close_hand_flag=true;
        notice_data_clear(&notice_message);
        notice_message.id=1;
        notice_message.data[0]=14;
        notice_publisher.publish(notice_message);
    }
    if(notice_message.id==1 && notice_message.data[0]==0)//open flag
    {
        open_hand_flag=true;
        notice_data_clear(&notice_message);
        notice_message.id=1;
        notice_message.data[0]=14;
        notice_publisher.publish(notice_message);
    }

}

void notice_pub_sub::notice_sub_spinner(char set)
{
    if(set==1)
        notice_spinner->start();
    if(set==0)
        notice_spinner->stop();
}

void notice_pub_sub::notice_data_clear(id_data_msgs::ID_Data *test)
{
    test->id=0;
    for(int i=0;i<8;i++) test->data[i]=0;
}

//function declaration
void do_exit(int error);
void signal_handler(int signal);
void init();
void bhand_axis_force_warning(notice_pub_sub *notice_test,axisSensorData force,axisSensorData torque);
bhand *hand_test;

int main(int argc,char **argv)
{
    ros::init(argc,argv,"bhand_axis_force");
    ros::NodeHandle handle_test;
    init();

    string dev_name;
    ros::param::get("~dev_name",dev_name);
    ROS_INFO("PCAN Device:%s",dev_name.c_str());

    float bhand_open_pos=0.5;
    ros::param::get("~bhand_open_pos",bhand_open_pos);
    ROS_INFO("Bhand open position :%f",bhand_open_pos);

    float bhand_close_pos=1.5;
    ros::param::get("~bhand_close_pos",bhand_close_pos);
    ROS_INFO("Bhand close position :%f",bhand_close_pos);

    ros::param::get("~force_display_on",force_display_on);
    ROS_INFO("ForceData display:%s",force_display_on==false?"false":"true");

    ros::param::get("~force_threshold",force_threshold);
    ROS_INFO("Force threshold  :%f",force_threshold);
    ros::param::get("~torque_threshold",torque_threshold);
    ROS_INFO("Torque threshold  :%f",torque_threshold);

    ros::param::get("~force_danger",force_danger);
    ROS_WARN(" Force dangerous threshold :%f",force_danger);
    ros::param::get("~torque_danger",torque_danger);
    ROS_WARN("Torque dangerous threshold :%f",torque_danger);

    ros::param::get("~left_torque_x_threshold",left_torque_x_threshold);
    ROS_WARN(" left torque x threshold :%f",left_torque_x_threshold);
    ros::param::get("~right_torque_x_threshold",right_torque_x_threshold);
    ROS_WARN("right torque x threshold :%f",right_torque_x_threshold);

    bhand bhand_handle(dev_name);
    hand_test=&bhand_handle;
    //test of can send data
    bhand_handle.barett_hand_init();
    bhand_handle.axisForce_init();
    //velocity control
    float v=0.1;
    ROS_WARN("Velocity control test begin ...");
    bhand_handle.setJointVelocity(FINGER1,v);
    sleep(5);
    bhand_handle.setJointVelocity(FINGER1,0);
    ROS_WARN("Velocity control test end ...");

    ROS_WARN("Velocity control test begin ...");
    bhand_handle.setJointVelocity(FINGER1,-v);
    sleep(5);
    bhand_handle.setJointVelocity(FINGER1,0);
    ROS_WARN("Velocity control test end ...");

    bhand_handle.setJointPosition(0.5,0.5,0.5,0);//position: (0-2.5)*10 0000
    bhand_handle.wait_act_finish();//waiting for bhand ready...

    ros::Rate loop_rate(200);

    notice_pub_sub notice_test;
    id_data_msgs::ID_Data notice_data_pub;
    int sys_count=0;
    while(ros::ok())
    {
        sys_count++;
        bhand_handle.can_handle.read_loop(false);
        //HANDLE h,ros::Publisher pub, bool display_on,bool publish_on
        if(close_hand_flag)
        {
            open_hand_flag=false;
            bhand_handle.setJointPosition(bhand_close_pos,bhand_close_pos,bhand_close_pos,0);//position: (0-2.5)*10 0000
            bhand_handle.wait_act_finish();
            notice_test.notice_data_clear(&notice_data_pub);
            notice_data_pub.id=1;
            notice_data_pub.data[0]=2;
            notice_test.notice_pub_sub_pulisher(notice_data_pub);
            close_hand_flag=false;
            ROS_INFO("Close hand successfully!");
         }

        if(open_hand_flag)
        {
            close_hand_flag=false;
            bhand_handle.setJointPosition(bhand_open_pos-0.2,bhand_open_pos-0.2,bhand_open_pos-0.2,0);//position: (0-2.5)*10 0000
            bhand_handle.wait_act_finish();
            bhand_handle.setJointPosition(bhand_open_pos,bhand_open_pos,bhand_open_pos,0);//position: (0-2.5)*10 0000
            bhand_handle.wait_act_finish();
            notice_test.notice_data_clear(&notice_data_pub);
            notice_data_pub.id=1;
            notice_data_pub.data[0]=2;
            notice_test.notice_pub_sub_pulisher(notice_data_pub);
            open_hand_flag=false;
            ROS_INFO("Open hand successfully!");
        }

        if(sys_count%20==0)
        {
//            ROS_INFO("    F1 Position:%f",bhand_handle.enc_to_rad(bhand_handle.get_position(FINGER1)));
//            ROS_INFO("    F2 Position:%f",bhand_handle.enc_to_rad(bhand_handle.get_position(FINGER2)));
//            ROS_INFO("    F3 Position:%f",bhand_handle.enc_to_rad(bhand_handle.get_position(FINGER3)));
//            ROS_INFO("SPREAD Position:%f",bhand_handle.enc_to_rad(bhand_handle.get_position(SPREAD)));
//            ROS_INFO("Finger1 V:%f",bhand_handle.get_velocity(FINGER1));
            axisSensorData force_display    =bhand_handle.get_AxisForce (AxisForceQuery);
            axisSensorData torque_dispaly   =bhand_handle.get_AxisTorque(AxisTorqueQuery);
            bhand_axis_force_warning(&notice_test,force_display,torque_dispaly);
            bhand_handle.displayAxisForceData (force_display);
            bhand_handle.displayAxisTorqueData(torque_dispaly);
        }
        if(sys_count%400==0)
        {
            v=-v;
            bhand_handle.setJointVelocity(FINGER1,v);
        }
        notice_test.notice_sub_spinner(1);
        loop_rate.sleep();
    }



    bhand_handle.setJointVelocity(FINGER1,0);
    bhand_handle.setJointVelocity(FINGER2,0);
    bhand_handle.setJointVelocity(FINGER3,0);
    bhand_handle.setJointVelocity(SPREAD,0);
	return 0;
}



void bhand_axis_force_warning(notice_pub_sub *notice_test,axisSensorData force,axisSensorData torque)
{
    id_data_msgs::ID_Data notice_data_pub;
    float force_x,force_y,force_z,torque_x,torque_y,torque_z;
    force_x=force.force_x;
    force_y=force.force_y;
    force_z=force.force_z;
    torque_x=torque.torque_x;
    torque_y=torque.torque_y;
    torque_z=torque.torque_z;
    if(fabs(force_x)>force_threshold || fabs(force_y)>force_threshold || fabs(force_z)>force_threshold)
    {
        notice_test->notice_data_clear(&notice_data_pub);
        notice_data_pub.id=5;
        notice_data_pub.data[0]=13;
        notice_test->notice_pub_sub_pulisher(notice_data_pub);
        ROS_WARN(" Force threshhold:%8.4f BHAND COLLISION!",force_threshold);
        ROS_INFO(" Force x:%8.4f, y:%8.4f, z:%8.4f",force_x,force_y,force_z);
        if(torque_x<=left_torque_x_threshold )
        {
            notice_test->notice_data_clear(&notice_data_pub);
            notice_data_pub.id=5;
            notice_data_pub.data[0]=12;
            notice_test->notice_pub_sub_pulisher(notice_data_pub);
            ROS_WARN("LEFT arm Axis force sensor detect BHAND COLLISION!");
            usleep(10000);
        }
        if(torque_x>=right_torque_x_threshold )
        {
            notice_test->notice_data_clear(&notice_data_pub);
            notice_data_pub.id=5;
            notice_data_pub.data[0]=11;
            notice_test->notice_pub_sub_pulisher(notice_data_pub);
            ROS_WARN("RIGHT Axis force sensor detect BHAND COLLISION!");
            usleep(10000);
        }

    }

    if(fabs(torque_x)>torque_threshold || fabs(torque_y)>torque_threshold || fabs(torque_z)>torque_threshold)
    {
        notice_test->notice_data_clear(&notice_data_pub);
        notice_data_pub.id=5;
        notice_data_pub.data[0]=13;
        notice_test->notice_pub_sub_pulisher(notice_data_pub);
        ROS_WARN("Torque threshhold:%8.4f BHAND COLLISION!",torque_threshold);
        ROS_INFO("Torque x:%8.4f, y:%8.4f, z:%8.4f",torque_x,torque_y,torque_z);
    }

    if(fabs(torque_x)>torque_danger || fabs(torque_y)>torque_danger || fabs(torque_z)>torque_danger || fabs(force_x)>force_danger || fabs(force_y)>force_danger || fabs(force_z)>force_danger)
    {
        notice_test->notice_data_clear(&notice_data_pub);
        notice_data_pub.id=5;
        notice_data_pub.data[0]=14;
        notice_test->notice_pub_sub_pulisher(notice_data_pub);
        ROS_ERROR("Axis force sensor detect BHAND DANGER!");
        ROS_INFO(" Force x:%8.4f, y:%8.4f, z:%8.4f",force_x,force_y,force_z);
        ROS_INFO("Torque x:%8.4f, y:%8.4f, z:%8.4f",torque_x,torque_y,torque_z);
    }

}

// exit handler
void do_exit(int error)
{   
    hand_test->setJointVelocity(FINGER1,0);
    hand_test->setJointVelocity(FINGER2,0);
    hand_test->setJointVelocity(FINGER3,0);
    hand_test->setJointVelocity(SPREAD,0);
    printf("Barett Hand: Stop moving.\n");
    printf("CAN Bus test: finished (%d).\n\n", error);
    //after call the target function in ELF object,close it using dlclose
    exit(error);
}

// the signal handler for manual break Ctrl-C
void signal_handler(int signal)
{
    do_exit(0);
}

// what has to be done at program start
void init()
{
    /* install signal handlers */
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
}
