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

using namespace std;
using namespace Eigen;

class RobotKinematics
{   
    private:
        int mess;
        ros::NodeHandle n;
        // The following publish to the topics added in the Coppelia sim node using the ROSInterface helper customization script
        ros::Publisher start_pub;  
        ros::Publisher sync_pub;
        ros::Publisher trigger_pub;
        ros::Publisher pause_pub;
        // The following nodes are to interact with the robot model
        ros::Publisher joint_torque_pub;
        ros::Subscriber joint_sub;
      
        // declaring joint parameters and pi (fixed stuff)
        double d1;
        double d2;
        double d3;
        double pi;    
    public:
        void getjoint_cb (const sensor_msgs::JointState &currentjoint); // to be defined in robot_control.cpp library
        double c1,s1,c2,s2,c3,s3,c4,s4,c5,s5,c6,s6,c7,s7;
        double th1,th2,th3,th4,th5,th6,th7;
         RobotKinematics();
        ArrayXd calculateFKM();
            
};
#endif