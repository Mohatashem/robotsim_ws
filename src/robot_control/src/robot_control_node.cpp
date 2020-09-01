#include "robot_control/robot_control.h"
# include "std_msgs/String.h"



int main (int argc, char **argv)
{
    ros::init(argc, argv, "robot_control_node");

    RobotKinematics robo;
    std::cout << "robot_control_node is active" << endl;
    ros::spinOnce();
    return 0;

}

