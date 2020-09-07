#ifndef CONTROLLER_CLASS_H
#define CONTROLLER_CLASS_H

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
class ControllerClass
{ 	private:
		int mess_;
		ros::NodeHandle n_;
		ros::Publisher start_pub_;  
        ros::Publisher sync_pub_;
        ros::Publisher trigger_pub_;
        ros::Publisher pause_pub_;
		ros::Publisher robot_force_pub_;
        ros::Subscriber joint_sub_;
		ros::Subscriber forward_kinematics_subscriber_;

	public:

		ControllerClass();
		 void getforwardkinematicsCallback(const std_msgs::Float32MultiArray &getforwardkinematics);
		 void getjointstateCallback (const sensor_msgs::JointState &currentjointstate);
		 Matrix<double,6,1> x_msr_; Matrix<double,6,1>   x_cmd_;
		 Matrix<double,6,1> x_dot_msr_; Matrix<double,6,1>   x_dot_cmd_;		 
		 Matrix<double,7,1>  q_msr_;
		 Matrix<double,6,1> cartesianpos_error_;
		 Matrix<double,6,1> cartesianvel_error_;

		 Matrix<double,7,1> jointpos_error_; // depends on your robot DOF
		 Matrix<double,7,1> jointvel_error_; // depends on your robot DOF
		 Matrix<double,6,1> previous_cartesianpos_error_;
		 Matrix<double,6,1> previous_cartesianvel_error_;
		 Matrix<double,6,1> force_impedance;
		 double KPx_, KPy_, KPz_, KPa_, KPb_, KPc_;
		 double KDx_, KDy_, KDz_, KDa_, KDb_, KDc_;
		 std_msgs::Float32 robot_[6];
		 std_msgs::Float32MultiArray robot_go_;
		 Matrix<double,7,1> joint_null_space_torque_;
		 Matrix<double,7,1> grav_compensation_torque_;
		 Matrix<double,6,1> cartesianImpedancecontroller(Matrix<double,6,1> cartesianpos_error_,Matrix<double,6,1> cartesianvel_error_);


};
#endif