#include "robot_control/robot_control.h"


class ControllerClass
{ private:
		ros::NodeHandle n_;

	public:
		 void getforwardkinematicsCallback(const std_msgs::Float32MultiArray &getforwardkinematics);
		 void getinversekinematicsCallback(const std_msgs::Float32MultiArray &getforwardkinematics);
		 void getjointstateCallback (const sensor_msgs::JointState &currentjointstate);
		 Array<double,6,1> cartesianpos_error_;
		 Array<double,6,1> cartesianvel_error_;
		 Array<double,7,1> jointpos_error_; // depends on your robot DOF
		 Array<double,7,1> jointvel_error_; // depends on your robot DOF
		 Array<double,6,1> previous_cartesianpos_error;
		 Array<double,6,1> previous_cartesianvel_error;



};
