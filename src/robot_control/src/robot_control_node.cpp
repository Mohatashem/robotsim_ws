#include "robot_control/robot_control.h"
# include "std_msgs/String.h"



// Just a testing the ROS node based on code from:
// https://github.com/therobocademy/ROS_Learning_Path_A/blob/master/Class_Notes/Day_12-ROS_Programming_Part1/code/hello_world/src/talker.cpp

// to be replaced the controller Node code.
int main (int argc, char **argv)
{
    ros::init(argc, argv, "robot_control_node");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter",10);
    ros::Rate loop_rate(10);
    int count = 0;




    while(ros::ok())
    {
        std_msgs::String msg;
        std::string message = "hello visual studio";
        msg.data = message;
        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;

}

