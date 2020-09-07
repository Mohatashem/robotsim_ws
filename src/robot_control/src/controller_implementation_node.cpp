#include "robot_control/robot_control.h"
#include "robot_control/controller_class.h"

# include "std_msgs/String.h"



int main (int argc, char **argv)
{
    ros::init(argc, argv, "controller_implementation_node");

    ControllerClass torquecontrollers;

    while (ros::ok()){
      cout <<"node is running"<< endl;
      cout << torquecontrollers.x_msr_ << endl;


      ros::spinOnce();

    }
    


    return 0;

}

