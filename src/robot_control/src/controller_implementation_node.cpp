#include "robot_control/controller_class.h"

# include "std_msgs/String.h"



int main (int argc, char **argv)
{
    ros::init(argc, argv, "controller_implementation_node");
    ControllerClass torquecontrollers;
    ros::Rate loop_rate(200); // 5ms

    while (ros::ok()){
      cout << "Jacobian is" << endl;
      std::cout << torquecontrollers.analytical_jacobian_ << endl;
      ros::spinOnce();
      loop_rate.sleep();
    }
    


    return 0;

}

