#include "robot_control/robot_control.h"
#include "ros/ros.h"
#include <iostream>


int main (int argc, char const *argv[])
{
    cout<< "hello control" << endl;
    RobotKinematics oj;
    ArrayXd x_msr = oj.calculateFKM();
    cout<< "x_msr = " << x_msr << endl;
    cout << "idiot vscode" << endl; 
    return 0;

}