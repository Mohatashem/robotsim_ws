#include "robot_control/robot_control.h"
#include "ros/ros.h"
#include <iostream>



RobotKinematics::RobotKinematics():
    mess_(1),
    d1_(0.4), 
    d2_(0.39), 
    d3_(0.2), 
    pi_(3.1415926535898)
    //th1_(0.57)
{    
    start_pub_ = n_.advertise<std_msgs::Bool>("startSimulation", mess_);
    sync_pub_ = n_.advertise<std_msgs::Bool>("enableSyncMode", mess_);
    trigger_pub_ = n_.advertise<std_msgs::Bool>("triggerNextStep", mess_);
    pause_pub_ = n_.advertise<std_msgs::Bool>("pauseSimulation",mess_);

    joint_torque_pub_ = n_.advertise<sensor_msgs::JointState>("set_jointforce", mess_);
    joint_sub_ = n_.subscribe("get_jointState",mess_,&RobotKinematics::getjointCallback,this);
    orientation_sub_ = n_.subscribe("get_orientation",mess_,&RobotKinematics::orientationCallback,this);
}
ArrayXd RobotKinematics::calculateFKM(Array<double,7,1> q_msr_)
{
                 
    c1_ = cos(q_msr_(0));s1_=sin(q_msr_(0));c2_ = cos(q_msr_(1));s2_=sin(q_msr_(1));
    c3_ = cos(q_msr_(2));s3_=sin(q_msr_(2));c4_ = cos(q_msr_(3));s4_=sin(q_msr_(3));
    c5_= cos(q_msr_(4));s5_=sin(q_msr_(4));c6_ = cos(q_msr_(5));s6_=sin(q_msr_(5));
    c7_ = cos(q_msr_(6));s7_=sin(q_msr_(6));

    double T07val[7][4][4] = {

            {
                {c1_,0,s1_,0},
                {s1_,0,-c1_,0},
                {0,1,0,0},
                {0,0,0,1}
            },
            {
                {c2_,0,-s2_,0},
                {s2_,0,c2_,0},
                {0,-1,0,0},
                {0,0,0,1}
            },

            {
                {c3_,0,-s3_,0},
                {s3_,0,c3_,0},
                {0,-1,0,d1_},
                {0,0,0,1}
            },
            {
                {c4_,0,s4_,0},
                {s4_,0,-c4_,0},
                {0,1,0,0},
                {0,0,0,1}
            },
            {
                {c5_,0,s5_,0},
                {s5_,0,-c5_,0},
                {0,1,0,d2_},
                {0,0,0,1}
            },
            {
                {c6_,0,-s6_,0},
                {s6_,0,c6_,0},
                {0,-1,0,0},
                {0,0,0,1}
            },
            {
                {c7_,-s7_,0,0},   
                {s7_,c7_,0,0},
                {0,0,1,d3_},
                {0,0,0,1}
            }
        };
    Matrix4d T07;
    for (int i = 0; i<7;i++)
    {
        for(int j=0;j<4;j++)
        {
            for(int k=0;k<4;k++)
            {
                T07(j,k) = T07val[i][j][k]; 
            }
        }
        T07 *= T07;
    }  // exiting the for loop we have total transformation from base to EE.
    cout <<T07<<endl;
    x_msr_ = (T07.col(3)).head(3);

    
    //return x_msr_;
};


void RobotKinematics::getjointCallback(const sensor_msgs::JointState &currentjointstate)
{
  for(int i=0;i<7;i++)
  {
    q_msr_(i,0) = currentjointstate.position[i];
       
  }
   // ArrayXd x;
    //x = calculateFKM(q_msr_);
    
/*
    sensor_msgs::JointState joint_cmd_torque;
    joint_cmd_torque.name.resize(7);
	joint_cmd_torque.position.resize(7);
	joint_cmd_torque.velocity.resize(7);
	joint_cmd_torque.effort.resize(7);
    joint_cmd_torque.name[0] = "joint_1";
    joint_cmd_torque.name[1] = "joint_2";
    joint_cmd_torque.name[2] = "joint_3";
    joint_cmd_torque.name[3] = "joint_4";
    joint_cmd_torque.name[4] = "joint_5";
    joint_cmd_torque.name[5] = "joint_6";
    joint_cmd_torque.name[6] = "joint_7";
    joint_cmd_torque.header.stamp = ros::Time::now();//check comment it work or
    for (int i = 0; i < 7; i++)
			  {
				
				  joint_cmd_torque.effort[i] = 50*sin(i*3.14);

			  }

    joint_torque_pub_.publish(joint_cmd_torque);
*/
    
};

void RobotKinematics::orientationCallback(const std_msgs::Float32MultiArray &getorientation)
{
  for(int i=0;i<3;i++)
	    {
		 	 angle_(i,0) = getorientation.data[i];
	    }


}