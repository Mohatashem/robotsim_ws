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

#include "robot_control/controller_class.h"


ControllerClass::ControllerClass():

    d1_(0.4),d2_(0.39),d3_(0.0),pi_(3.1415926535898),
    mess_(1),KPx_(50), KPy_(50), KPz_(50), KPa_(0.0050), KPb_(0.0050), KPc_(0.0050),
	KDx_(0.01), KDy_(0.01), KDz_(0.01), KDa_(0.001), KDb_(0.001), KDc_(0.001)

{
  /* start_pub_ = n_.advertise<std_msgs::Bool>("startSimulation", mess_);
    sync_pub_ = n_.advertise<std_msgs::Bool>("enableSyncMode", mess_);
    trigger_pub_ = n_.advertise<std_msgs::Bool>("triggerNextStep", mess_);
    pause_pub_ = n_.advertise<std_msgs::Bool>("pauseSimulation",mess_);*/
    simtime_ = n_.subscribe("simulationTime",mess_,&ControllerClass::simtime_cb,this);
    forward_kinematics_subscriber_ = n_.subscribe("get_forward_kinematics",mess_,&ControllerClass::getforwardkinematicsCallback,this);
    joint_state_pub_ = n_.advertise<sensor_msgs::JointState>("set_jointstate", mess_);
    robot_force_pub_ = n_.advertise<std_msgs::Float32MultiArray>("set_robotforce", mess_);

};

void ControllerClass::simtime_cb(const std_msgs::Float32 &msg)
{
    ts_ = msg.data;
}

void ControllerClass::getforwardkinematicsCallback(const std_msgs::Float32MultiArray &getforwardkinematics)
{
    for(int i=0;i<6;i++)
	    {
		 	 x_msr_(i,0) = getforwardkinematics.data[i];
	    }
    x_cmd_<< -0.54,0.0,-0.218,-3.14, 0.005,-3.14;
    x_dot_cmd_ << 0.0,0.0,0.0,0.0,0.0,0.0;
    x_dot_msr_ = (x_msr_ - prev_x_msr_)/0.005;

    for(int i=0;i<6;i++)
    {
        if(x_dot_msr_(i,0) == 0)
        {
            x_dot_msr_(i,0)= prev_x_dot_msr_(i,0);
        }
        if(x_dot_msr_(i) > 2 || x_dot_msr_(i) < -2)
        {
            x_dot_msr_(i) = 0.0;
        }
    }
    
    cartesianpos_error_ = x_cmd_ - x_msr_;
    cartesianvel_error_ = x_dot_cmd_ - x_dot_msr_;

    analytical_jacobian_ = calculateJacobianMatrix(q_msr_);
    force_impedance = cartesianImpedancecontroller(cartesianpos_error_,cartesianvel_error_);
    

    total_torque_ = analytical_jacobian_.transpose()*force_impedance;
 /*  for (int i = 0; i < 6; i++)
            {
                robot_[i].data = force_total(i,0);
                robot_go_.data.push_back(robot_[i].data);
                

            }*/
    for (int i = 0; i < 7; i++)
			  {
				  double sign = 1.0;
				  if(total_torque_(i) < 0.0)
				  {
					  sign = -1.0;
				  }

				  joint_state_.effort.push_back(fabs(total_torque_(i))); 
				  joint_state_.velocity.push_back(sign*10000.0);
                  joint_state_.name.push_back("joint_"+std::to_string(i+1));
			  } 

    joint_state_pub_.publish(joint_state_);
    prev_x_msr_ = x_msr_;
    prev_x_dot_msr_ =x_dot_msr_;      
    //sync_pub.publish(synch);
    // start_pub.publish(start);


 //   robot_force_pub_.publish(robot_go_);




}

ArrayXd ControllerClass::calculateFKM(Array<double,7,1> q_msr_)
{
                 
    c1_ = cos(q_msr_(0));s1_=sin(q_msr_(0));c2_ = cos(q_msr_(1));s2_=sin(q_msr_(1));
    c3_ = cos(q_msr_(2));s3_=sin(q_msr_(2));c4_ = cos(q_msr_(3));s4_=sin(q_msr_(3));
    c5_= cos(q_msr_(4));s5_=sin(q_msr_(4));c6_ = cos(q_msr_(5));s6_=sin(q_msr_(5));
    c7_ = cos(q_msr_(6));s7_=sin(q_msr_(6));

    double T07val[7][4][4] = {

            {
                {c1_,0,s1_,0},
                {s1_,0,-c1_,0},
                {0,1,0,0},
                {0,0,0,1}
            },
            {
                {c2_,0,-s2_,0},
                {s2_,0,c2_,0},
                {0,-1,0,0},
                {0,0,0,1}
            },

            {
                {c3_,0,-s3_,0},
                {s3_,0,c3_,0},
                {0,-1,0,d1_},
                {0,0,0,1}
            },
            {
                {c4_,0,s4_,0},
                {s4_,0,-c4_,0},
                {0,1,0,0},
                {0,0,0,1}
            },
            {
                {c5_,0,s5_,0},
                {s5_,0,-c5_,0},
                {0,1,0,d2_},
                {0,0,0,1}
            },
            {
                {c6_,0,-s6_,0},
                {s6_,0,c6_,0},
                {0,-1,0,0},
                {0,0,0,1}
            },
            {
                {c7_,-s7_,0,0},   
                {s7_,c7_,0,0},
                {0,0,1,d3_},
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
    x_msr_ = (T07.col(3)).head(3);

    
    //return x_msr_;
};


Matrix<double,6,7> ControllerClass::calculateJacobianMatrix(Array<double,7,1> q_msr_)
{
    c1_ = cos(q_msr_(0));s1_=sin(q_msr_(0));c2_ = cos(q_msr_(1));s2_=sin(q_msr_(1));
    c3_ = cos(q_msr_(2));s3_=sin(q_msr_(2));c4_ = cos(q_msr_(3));s4_=sin(q_msr_(3));
    c5_= cos(q_msr_(4));s5_=sin(q_msr_(4));c6_ = cos(q_msr_(5));s6_=sin(q_msr_(5));
    c7_ = cos(q_msr_(6));s7_=sin(q_msr_(6));

    double T07val[7][4][4] = {

            {
                {c1_,0,s1_,0},
                {s1_,0,-c1_,0},
                {0,1,0,0},
                {0,0,0,1}
            },
            {
                {c2_,0,-s2_,0},
                {s2_,0,c2_,0},
                {0,-1,0,0},
                {0,0,0,1}
            },

            {
                {c3_,0,-s3_,0},
                {s3_,0,c3_,0},
                {0,-1,0,d1_},
                {0,0,0,1}
            },
            {
                {c4_,0,s4_,0},
                {s4_,0,-c4_,0},
                {0,1,0,0},
                {0,0,0,1}
            },
            {
                {c5_,0,s5_,0},
                {s5_,0,-c5_,0},
                {0,1,0,d2_},
                {0,0,0,1}
            },
            {
                {c6_,0,-s6_,0},
                {s6_,0,c6_,0},
                {0,-1,0,0},
                {0,0,0,1}
            },
            {
                {c7_,-s7_,0,0},   
                {s7_,c7_,0,0},
                {0,0,1,d3_},
                {0,0,0,1}
            }
        };


    Matrix4d T01,T12,T23,T34,T45,T56,T67;
    Matrix4d T02,T03,T04,T05,T06,T07;
    for(int j=0;j<4;j++)
    {
           for(int k=0;k<4;k++)
           {
               T01(j,k) = T07val[0][j][k];
               T12(j,k) = T07val[1][j][k];
               T23(j,k) = T07val[2][j][k];
               T34(j,k) = T07val[3][j][k];
               T45(j,k) = T07val[4][j][k];
               T56(j,k) = T07val[5][j][k];
               T67(j,k) = T07val[6][j][k];
           }
    }
    T02 = T01*T12;T03=T02*T23;T04=T03*T34;T05=T04*T45;T06=T05*T56;T07 = T06*T67;
    Matrix<double,3,1> P0,P1,P2,P3,P4,P5,P6;
    Matrix<double,3,1> z0,z1,z2,z3,z4,z5,z6;
    Matrix<double,3,1> Pe,Pe0,Pe1,Pe2,Pe3,Pe4,Pe5,Pe6;
    Matrix<double,3,1> U0,U1,U2,U3,U4,U5,U6;
    Matrix<double,6,1> J1,J2,J3,J4,J5,J6,J7;
    Matrix<double,6,7> Jg;
    Matrix<double,6,7> JA;
    Matrix<double,3,3> Bmat_,Binvmat_;
    Matrix<double,6,6> BigB_mat_;
    MatrixXd Zero3 = Matrix3d::Zero(3,3);
    MatrixXd I3 = Matrix3d::Identity(3,3);

    P0<<0,0,1; z0 <<0,0,1;
    for(int i=0;i<3;i++) 
    {// 0,1,2, CORRESPONDING TO ROW 1,2,3,

        P1(i,0) = T01(i,3);
        P2(i,0) = T02(i,3);
        P3(i,0) = T03(i,3);
        P4(i,0) = T04(i,3);
        P5(i,0) = T05(i,3);
        P6(i,0) = T06(i,3);
        Pe(i,0) = T07(i,3);
    }

    for(int i=0;i<3;i++) // 0,1,2 CORRESPONDING TO ROW 1,2,3
    {
        z1(i,0) = T01(i,2);
        z2(i,0) = T02(i,2);
        z3(i,0) = T03(i,2);
        z4(i,0) = T04(i,2);
        z5(i,0) = T05(i,2);
        z6(i,0) = T06(i,2);
    }

    Pe0= Pe - P0; // first three elements of the fourth column of the end-effector frame (last frame)
    Pe1= Pe - P1;
    Pe2= Pe - P2;
    Pe3= Pe - P3;
    Pe4= Pe - P4;
    Pe5= Pe - P5;
    Pe6= Pe - P6;

    U0 = z0.cross(Pe0);
    U1 = z1.cross(Pe1);
    U2 = z2.cross(Pe2);
    U3 = z3.cross(Pe3);
    U4 = z4.cross(Pe4);
    U5 = z5.cross(Pe5);
    U6 = z6.cross(Pe6);


    // Now we will create jacobian columns
    J1 << U0,z0;J2 << U1,z1;J3 << U2,z2;J4 << U3,z3;J5 << U4,z4;J6 << U5,z5;J7 << U6,z6;

    // concatenating the Columns of the Jacobian horizontally
    Jg << J1,J2,J3,J4,J5,J6,J7;

    cout<< Jg << endl;
    Bmat_ << 1.0,0.0,sin(x_msr_(4,0)),
			0.0,cos(x_msr_(3,0)),-cos(x_msr_(4,0))*sin(x_msr_(3,0)),
			0.0,sin(x_msr_(3,0)),cos(x_msr_(4,0))*cos(x_msr_(3,0));

    Binvmat_ = Bmat_.inverse();
    BigB_mat_ << I3,Zero3,
			  Zero3,Binvmat_;
    JA = BigB_mat_*Jg; 

    return JA;
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
    force_impedance = KP*cartesianpos_error_ - KD*cartesianvel_error_;

    return force_impedance; 

};