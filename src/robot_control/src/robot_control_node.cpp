#include "robot_control/robot_control.h"
# include "std_msgs/String.h"



int main (int argc, char **argv)
{
    ros::init(argc, argv, "robot_control_node");

    RobotKinematics robo;

    while (ros::ok()){
      cout <<"node is running"<< robo.q_msr_ << endl;
      ros::spinOnce();

    }
    


    return 0;

}

