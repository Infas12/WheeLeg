#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include "socketCan2/inc/CanManager.hpp"
#include "socketCan2/inc/DjiMotorManager.hpp"
#include "socketCan2/inc/M2006.hpp"
#include "socketCan2/inc/M3508.hpp"


M3508 ChassisMotor[4];
float Vy = 0.0;
float Vw = 0.0;


void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    Vw = 5.0 * joy->axes[0];
    Vy = 5.0 * joy->axes[1];
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "chassis");    
    ros::NodeHandle n;  
    ros::Subscriber joy_sub_ = n.subscribe<sensor_msgs::Joy>("joy", 10, &joystickCallback); 
    ros::Rate loop_rate(1000);

    for(int i = 0; i < 4; i++)
    {
        ChassisMotor[i].Registration(0x201+i);
        ChassisMotor[i].pidSpeed.kp = 3;
	    ChassisMotor[i].pidSpeed.ki = 0.2;
	    ChassisMotor[i].pidSpeed.kd = 0.05;
	    ChassisMotor[i].pidSpeed.maxOut = 10;
	    ChassisMotor[i].pidSpeed.maxIOut = 10;
        ChassisMotor[i].controlMode = Motor::SPD_MODE;
    }

    CanManager::Instance()->SetPortName("can0");
    CanManager::Instance()->Init();

    while (ros::ok())
    {   

        ChassisMotor[0].speedSet = Vy - Vw; //RightFront
        ChassisMotor[1].speedSet = -(Vy + Vw); //LeftFront
        ChassisMotor[2].speedSet = Vy - Vw; //RightRear
        ChassisMotor[3].speedSet = -(Vy + Vw); //LeftRear
        
        for(int i = 0; i < 4; i++)
        {
            ChassisMotor[i].Update(); //Update Motor Control
        }

        DjiMotorManager::Instance()->Update();//Send Commands

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}