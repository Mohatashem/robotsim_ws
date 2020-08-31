#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

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

using namespace std;
using namespace Eigen;

class RobotKinematics
{   
    private:
        double d1;
        double d2;
        double d3;
        double pi;    
    public:
        double c1,s1,c2,s2,c3,s3,c4,s4,c5,s5,c6,s6,c7,s7;
        double th1,th2,th3,th4,th5,th6,th7;
        RobotKinematics(/*double d1, double d2, double d3*/)
        {   
            d1 = 0.15;
            d2 = 0.15;
            d3 = 0.1;
            pi = 3.1415926535898;
            th1 = 0.57;
        }

    ArrayXd calculateFKM()
    {              
        c1 = cos(th1);s1=sin(th1);c2 = cos(th1);s2=sin(th1);
        c3 = cos(th1);s3=sin(th1);c4 = cos(th1);s4=sin(th2);
        c5 = cos(th1);s5=sin(th1);c6 = cos(th1);s6=sin(th2);
        c7 = cos(th1);s7=sin(th1);

        double T07val[7][4][4] = {

                {
                    {c1,0,s1,0},
                    {s1,0,-c1,0},
                    {0,1,0,0},
                    {0,0,0,1}
                },
                {
                    {c2,0,-s2,0},
                    {s2,0,c2,0},
                    {0,-1,0,0},
                    {0,0,0,1}
                },

                {
                    {c3,0,-s3,0},
                    {s3,0,c3,0},
                    {0,-1,0,d1},
                    {0,0,0,1}
                },
                {
                    {c4,0,s4,0},
                    {s4,0,-c4,0},
                    {0,1,0,0},
                    {0,0,0,1}
                },
                {
                    {c5,0,s5,0},
                    {s5,0,-c5,0},
                    {0,1,0,d2},
                    {0,0,0,1}
                },
                {
                    {c6,0,-s6,0},
                    {s6,0,c6,0},
                    {0,-1,0,0},
                    {0,0,0,1}
                },
                {
                    {c7,-s7,0,0},   
                    {s7,c7,0,0},
                    {0,0,1,d3},
                    {0,0,0,1}
                }
            };
        Matrix4d T07;
        for (int i = 0; i<7;i++)
        {
            for(int j=0;j<4;j++)
            {
                for(int k=0;k<4;k++)
                {
                    T07(j,k) = T07val[i][j][k]; 
                }
            }
            T07 *= T07;
        }  // exiting the for loop we have total transformation from base to EE.
        cout <<T07<<endl;
        ArrayXd x_msr;
        x_msr = (T07.col(3)).head(3);
        return x_msr;
    }    
};
#endif