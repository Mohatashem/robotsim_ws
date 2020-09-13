/*
 * This file is part of robot_control package.
 *
 * This product includes software developed by Mohatashem Reyaz Makhdoomi
 * (https://github.com/Mohatashem).
 * See the COPYRIGHT file at the top-level directory of this distribution
 * for details of code ownership.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef CONTROLLER_CLASS_H
#define CONTROLLER_CLASS_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string> 
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32MultiArray.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/SVD"
#include "math.h"
#include "vector"

using std::cout;
using std::endl;
using namespace Eigen; // ask lentin
class ControllerClass
{ 	private:
		int mess_;
		ros::NodeHandle n_;
		/*ros::Publisher start_pub_;  
        ros::Publisher sync_pub_;
        ros::Publisher trigger_pub_;
        ros::Publisher pause_pub_;*/
		ros::Publisher joint_state_pub_;
		ros::Publisher robot_force_pub_;
        ros::Subscriber joint_sub_;
		ros::Subscriber forward_kinematics_subscriber_;
		ros::Subscriber simtime_;

	public:

		ControllerClass();
		void simtime_cb(const std_msgs::Float32 &msg);
		void getforwardkinematicsCallback(const std_msgs::Float32MultiArray &getforwardkinematics);
		void getjointstateCallback (const sensor_msgs::JointState &currentjointstate);
		double d1_;double d2_;double d3_;double pi_;
		double c1_,s1_,c2_,s2_,c3_,s3_,c4_,s4_,c5_,s5_,c6_,s6_,c7_,s7_;
		Matrix<double,6,1> x_msr_; Matrix<double,6,1> prev_x_msr_;Matrix<double,6,1>   x_cmd_;
		Matrix<double,6,1> x_dot_msr_; Matrix<double,6,1> prev_x_dot_msr_; Matrix<double,6,1>   x_dot_cmd_;		 
		Matrix<double,7,1>  q_msr_;
		Matrix<double,6,1> cartesianpos_error_;
		Matrix<double,6,1> cartesianvel_error_;
		double ts_; // simulation time
		Matrix<double,7,1> jointpos_error_; // depends on your robot DOF
		Matrix<double,7,1> jointvel_error_; // depends on your robot DOF
		Matrix<double,6,1> previous_cartesianpos_error_;
		Matrix<double,6,1> previous_cartesianvel_error_;
		sensor_msgs::JointState joint_state_;


		Matrix<double,6,1> force_impedance;
		double KPx_, KPy_, KPz_, KPa_, KPb_, KPc_;
		double KDx_, KDy_, KDz_, KDa_, KDb_, KDc_;
		std_msgs::Float32 robot_[6];
		std_msgs::Float32MultiArray robot_go_;
	
		Matrix<double,7,1> joint_null_space_torque_;
		Matrix<double,7,1> grav_compensation_torque_;
		Matrix<double,7,1> total_torque_;

		ArrayXd calculateFKM(Array<double,7,1> q_msr_);
		Matrix<double,6,7> calculateJacobianMatrix(Array<double,7,1> q_msr_);
		Matrix<double,6,7> analytical_jacobian_;
		Matrix<double,6,1> cartesianImpedancecontroller(Matrix<double,6,1> cartesianpos_error_,Matrix<double,6,1> cartesianvel_error_);


};
#endif