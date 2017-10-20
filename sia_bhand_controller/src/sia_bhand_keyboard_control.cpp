#include <stdio.h>
#include <linux/input.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ros/callback_queue.h"
using namespace std;
/*****************************************************************************************
 *debug note:
 *if there are some abnormal phenomenon,please run the command "sudo chmod 777 /dev/input/event3"
 *well suitable to the bhand_force_control.cpp of 2016,11,18 17:00.
 *date:2016,11,18,17:00
 * ***************************************************************************************/
#define HANDLE void *
#define DEV_PATH "/dev/input/event3" //cat /proc/bus/input/devices Name="AT Translated Set 2 keyboard"

//globals
float key_control_value=0;

class keyboard_monitor
{
public:
    boost::function<void (const geometry_msgs::Twist::ConstPtr&)> keyboard_monitor_CallbackFun;
    keyboard_monitor();
    void keyboard_monitor_spinner(char set);
private:
    ros::NodeHandle keyboard_handle;
    ros::Subscriber keyboard_subscriber;
    ros::SubscribeOptions keyboard_ops;
    ros::AsyncSpinner *keyboard_spinner;
    ros::CallbackQueue keyboard_callbackQueue;
    void keyboard_monitor_callback(const geometry_msgs::Twist::ConstPtr &key_control_msg);
};

keyboard_monitor::keyboard_monitor()
{
   keyboard_monitor_CallbackFun=boost::bind(&keyboard_monitor::keyboard_monitor_callback,this,_1);
   keyboard_ops=ros::SubscribeOptions::create<geometry_msgs::Twist>(
               "/keyboard_control",
               1,
               keyboard_monitor_CallbackFun,
               ros::VoidPtr(),
               &keyboard_callbackQueue
               );
   keyboard_subscriber=keyboard_handle.subscribe(keyboard_ops);
   keyboard_spinner=new ros::AsyncSpinner(1,&keyboard_callbackQueue);

}

void keyboard_monitor::keyboard_monitor_spinner(char set)
{
    if(set==1)
        keyboard_spinner->start();
    if(set==0)
        keyboard_spinner->stop();

}

void keyboard_monitor::keyboard_monitor_callback(const geometry_msgs::Twist::ConstPtr &key_control_msg)
{
    ROS_INFO("Keyboard control speed:%f",key_control_msg->linear.x);
    key_control_value=key_control_msg->linear.x;
}



int main(int argc, char **argv)
{
    int  keyboard_handle;
    input_event t;

    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle n;
    ros::Publisher key_pub=n.advertise<geometry_msgs::Twist>("keyboard_control",1);
    ros::Rate loop_rate(100);
    geometry_msgs::Twist msg;

    keyboard_handle=open(DEV_PATH, O_RDONLY);

    //judge whether the input event opened successfully
    if(keyboard_handle<=0)
    {
        printf("open %s error!\nPlease enter [sudo chmod 777 /dev/input/event3] in current terminal.\n",DEV_PATH);
        return -1;
    }
    else
        printf("open %s successfully!\n",DEV_PATH);

    printf("\nControl your Bhand BH280:\n---------------------\npress O: open all fingers \npress C: close all fingers\npress Q/Z increase/decrease max speeds by 10%\n\n");

    char o_pressed_flag;
    char c_pressed_flag;

    keyboard_monitor key_sub;


    while(ros::ok())
    {

        if(read(keyboard_handle, &t, sizeof(t)) == sizeof(t))
        {
            if(t.type==EV_KEY)
                if(t.value==0 || t.value==1)
                {
                    //.code-which key;.value 0:released,1:pressed
                    if(t.value==1 && t.code==KEY_C) o_pressed_flag=1;
                    while(o_pressed_flag)
                    {
//                        if(msg.linear.x>=0 && msg.linear.x<2.4) msg.linear.x+=0.01;
//                        else
//                            msg.linear.x=2.39;
                        msg.linear.x=0.01;
                        printf("linear speed x:%f\n",msg.linear.x);
                        key_pub.publish(msg);
                        if(read(keyboard_handle, &t, sizeof(t)) == sizeof(t))
                        {
                            if(t.type==EV_KEY)
                                if(t.value==0 || t.value==1)
                                {
                                    if(t.value==0 ) o_pressed_flag=0;
                                }
                        }
                        loop_rate.sleep();
                    }

                    if(t.value==1 && t.code==KEY_O) c_pressed_flag=1;
                    while(c_pressed_flag)
                    {
//                        if(msg.linear.x>0 && msg.linear.x<=2.4) msg.linear.x-=0.01;
//                        else
//                            msg.linear.x=0;
                        msg.linear.x=-0.01;
                        printf("linear speed x:%f\n",msg.linear.x);
                        key_pub.publish(msg);
                        if(read(keyboard_handle, &t, sizeof(t)) == sizeof(t))
                        {
                            if(t.type==EV_KEY)
                                if(t.value==0 || t.value==1)
                                {
                                    if(t.value==0 ) c_pressed_flag=0;
                                }
                        }
                        loop_rate.sleep();
                    }
                }
        }
        key_sub.keyboard_monitor_spinner(1);

        ros::spinOnce();
        loop_rate.sleep();
    }
    close(keyboard_handle);
    return 0;
}




