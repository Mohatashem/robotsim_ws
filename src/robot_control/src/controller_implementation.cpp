#include "robot_control/robot_control.h"
#include "robot_control/controller_class.h"


ControllerClass::ControllerClass():

    mess_(10),KPx_(100), KPy_(100), KPz_(100), KPa_(0.10), KPb_(0.10), KPc_(0.10),
	KDx_(0.01), KDy_(0.01), KDz_(0.01), KDa_(0.001), KDb_(0.001), KDc_(0.001)

{
    start_pub_ = n_.advertise<std_msgs::Bool>("startSimulation", mess_);
    sync_pub_ = n_.advertise<std_msgs::Bool>("enableSyncMode", mess_);
    trigger_pub_ = n_.advertise<std_msgs::Bool>("triggerNextStep", mess_);
    pause_pub_ = n_.advertise<std_msgs::Bool>("pauseSimulation",mess_);
    forward_kinematics_subscriber_ = n_.subscribe("get_forward_kinematics",mess_,&ControllerClass::getforwardkinematicsCallback,this);
    robot_force_pub_ = n_.advertise<std_msgs::Float32MultiArray>("set_robotforce", mess_);

};

void ControllerClass::getforwardkinematicsCallback(const std_msgs::Float32MultiArray &getforwardkinematics)
{
    for(int i=0;i<6;i++)
	    {
		 	 x_msr_(i,0) = getforwardkinematics.data[i];
	    }
        x_cmd_<< -0.54,0.0,-0.218,-3.14, 0.005,-3.14;
        x_dot_cmd_ << 0.0,0.0,0.0,0.0,0.0,0.0;
        x_dot_msr_ << 0.0,0.0,0.0,0.0,0.0,0.0;
        cartesianpos_error_ = x_cmd_ - x_msr_;
        cartesianvel_error_ = x_dot_cmd_ - x_dot_msr_;
        force_impedance = cartesianImpedancecontroller(cartesianpos_error_,cartesianvel_error_);
        Matrix<double,6,1> force_total = force_impedance;
        cout << "force total" <<force_total << endl;
        for (int i = 0; i < 6; i++)
			  {
				  

				  robot_[i].data = force_total(i,0);
                  robot_go_.data.push_back(robot_[i].data);
				  

			  }
			  
			  //sync_pub.publish(synch);
			 // start_pub.publish(start);


			  robot_force_pub_.publish(robot_go_);




}


Matrix<double,6,1> ControllerClass::cartesianImpedancecontroller(Matrix<double,6,1> cartesianpos_error_ ,Matrix<double,6,1> cartesianvel_error_)
{

    MatrixXd KP(6,6);
                KP <<KPx_,0,0,0,0,0,
                    0,KPy_,0,0,0,0,
                    0,0,KPz_,0,0,0,
                    0,0,0,KPa_,0,0,
                    0,0,0,0,KPb_,0,
                    0,0,0,0,0,KPc_;


                MatrixXd KD(6,6);

                KD <<KDx_,0,0,0,0,0,
                    0,KDy_,0,0,0,0,
                    0,0,KDz_,0,0,0,
                    0,0,0,KDa_,0,0,
                    0,0,0,0,KDb_,0,
                    0,0,0,0,0,KDc_;
    
    //force_impedance = KP*cartesianpos_error_ - KD*cartesianvel_error_;

    return KP*cartesianpos_error_ - KD*cartesianvel_error_;

};