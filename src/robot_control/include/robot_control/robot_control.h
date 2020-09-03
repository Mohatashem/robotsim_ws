#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "vector"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32MultiArray.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/SVD"

using std::cout;
using std::endl;
using namespace Eigen; // ask lentin

class RobotKinematics
{   
    private:
        int mess_;
        ros::NodeHandle n_;
        // The following publish to the topics added in the Coppelia sim node using the ROSInterface helper customization script
        ros::Publisher start_pub_;  
        ros::Publisher sync_pub_;
        ros::Publisher trigger_pub_;
        ros::Publisher pause_pub_;
        // The following nodes are to interact with the robot model
        ros::Publisher joint_torque_pub_;
        ros::Subscriber joint_sub_;
        ros::Subscriber orientation_sub_;
      
        // declaring joint parameters and pi (fixed stuff)
        double d1_;
        double d2_;
        double d3_;
        double pi_;    
    public:
        void getjointCallback (const sensor_msgs::JointState &currentjointstate); // to be defined in robot_control.cpp library
        void orientationCallback(const std_msgs::Float32MultiArray &getorientation);
        Array<double,7,1> q_msr_; // getting the joint variables from the robot
        Array<double,3,1>  x_msr_; Array<double,3,1>  x_cmd_; Array<double,3,1>  angle_;
        double c1_,s1_,c2_,s2_,c3_,s3_,c4_,s4_,c5_,s5_,c6_,s6_,c7_,s7_;
        RobotKinematics();
        ArrayXd calculateFKM(Array<double,7,1> q_msr_);
            
};
#endif