#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);
MatrixXd I15 = MatrixXd::Identity(15,15);

MatrixXd x(15,1); // predicted u at t
MatrixXd f(15,1);
MatrixXd cov = MatrixXd::Identity(15,15);


bool cam_ready=0;
double cur_t=0.0;

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    //your code for propagation
    
	if(!cam_ready)
	{
		cov = cov*100;
	}
	
	else
	{
	    double dT = msg->header.stamp.toSec()-cur_t;
	    cur_t = msg->header.stamp.toSec();
	    
	    if(dT<1)
	    {
			double x2x = x(3,0); // phi angle
			double x2y = x(4,0); // theta angle
			double x2z = x(5,0); // kesi angle
			double wmx = msg->angular_velocity.x;
			double wmy = msg->angular_velocity.y;
			double wmz = msg->angular_velocity.z;
			double amx = msg->linear_acceleration.x;
			double amy = msg->linear_acceleration.y;
			double amz = msg->linear_acceleration.z;
			double x4x = x(9,0);
			double x4z = x(11,0);
			double x5x = x(12,0);
			double x5y = x(13,0);
			double x5z = x(14,0);

			MatrixXd Ft(15,15);
			MatrixXd Vt(15,12);
			
			MatrixXd At(15,15);
			At<<0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                     0, 1, 0, 0,                                                                                                    0,  0,                                                                                                   0,                                                                              0,                              0,                                                                              0,
				0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                     0, 0, 1, 0,                                                                                                    0,  0,                                                                                                   0,                                                                              0,                              0,                                                                              0,
				0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                     0, 0, 0, 1,                                                                                                    0,  0,                                                                                                   0,                                                                              0,                              0,                                                                              0,
				0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                           (cos(conj(x2y))*(wmz - x4z))/(cos(conj(x2y))*cos(conj(x2y)) + sin(conj(x2y))*sin(conj(x2y))) - (sin(conj(x2y))*(wmx - x4x))/(cos(conj(x2y))*cos(conj(x2y)) + sin(conj(x2y))*sin(conj(x2y))),                                                                                                                                                                                                                                     0, 0, 0, 0,                                                -cos(conj(x2y))/(cos(conj(x2y))*cos(conj(x2y)) + sin(conj(x2y))*sin(conj(x2y))),  0,                                               -sin(conj(x2y))/(cos(conj(x2y))*cos(conj(x2y)) + sin(conj(x2y))*sin(conj(x2y))),                                                                              0,                              0,                                                                              0,
				0, 0, 0, (cos(conj(x2x))*sin(conj(x2y))*(wmx - x4x))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))) - (cos(conj(x2x))*cos(conj(x2y))*(wmz - x4z))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))) - (cos(conj(x2y))*sin(conj(x2x))*(sin(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + sin(conj(x2x))*sin(conj(x2y))*sin(conj(x2y)))*(wmz - x4z))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y)))*(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))) + (sin(conj(x2x))*sin(conj(x2y))*(sin(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + sin(conj(x2x))*sin(conj(x2y))*sin(conj(x2y)))*(wmx - x4x))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y)))*(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))), (cos(conj(x2y))*sin(conj(x2x))*(wmx - x4x))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))) + (sin(conj(x2x))*sin(conj(x2y))*(wmz - x4z))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))),                                                                                                                                                                                                                                     0, 0, 0, 0, -(sin(conj(x2x))*sin(conj(x2y)))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))), -1, (cos(conj(x2y))*sin(conj(x2x)))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))),                                                                              0,                              0,                                                                              0,
				0, 0, 0,                                                                                                                                                                                                                                                                   (cos(conj(x2y))*(sin(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + sin(conj(x2x))*sin(conj(x2y))*sin(conj(x2y)))*(wmz - x4z))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y)))*(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))) - (sin(conj(x2y))*(sin(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + sin(conj(x2x))*sin(conj(x2y))*sin(conj(x2y)))*(wmx - x4x))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y)))*(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))),                             - (cos(conj(x2y))*(wmx - x4x))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))) - (sin(conj(x2y))*(wmz - x4z))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))),                                                                                                                                                                                                                                     0, 0, 0, 0,                   sin(conj(x2y))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))),  0,                 -cos(conj(x2y))/(cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2y)) + cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2y))),                                                                              0,                              0,                                                                              0,
				0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                   sin(conj(x2x))*sin(conj(x2z))*(amy - x5y) + cos(conj(x2x))*cos(conj(x2y))*sin(conj(x2z))*(amz - x5z) - cos(conj(x2x))*sin(conj(x2y))*sin(conj(x2z))*(amx - x5x),                                           (cos(conj(x2y))*cos(conj(x2z)) - sin(conj(x2x))*sin(conj(x2y))*sin(conj(x2z)))*(amz - x5z) - (cos(conj(x2z))*sin(conj(x2y)) + cos(conj(x2y))*sin(conj(x2x))*sin(conj(x2z)))*(amx - x5x), - (cos(conj(x2y))*sin(conj(x2z)) + cos(conj(x2z))*sin(conj(x2x))*sin(conj(x2y)))*(amx - x5x) - (sin(conj(x2y))*sin(conj(x2z)) - cos(conj(x2y))*cos(conj(x2z))*sin(conj(x2x)))*(amz - x5z) - cos(conj(x2x))*cos(conj(x2z))*(amy - x5y), 0, 0, 0,                                                                                                    0,  0,                                                                                                   0,   sin(conj(x2x))*sin(conj(x2y))*sin(conj(x2z)) - cos(conj(x2y))*cos(conj(x2z)),  cos(conj(x2x))*sin(conj(x2z)), - cos(conj(x2z))*sin(conj(x2y)) - cos(conj(x2y))*sin(conj(x2x))*sin(conj(x2z)),
				0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                   cos(conj(x2x))*cos(conj(x2z))*sin(conj(x2y))*(amx - x5x) - cos(conj(x2x))*cos(conj(x2y))*cos(conj(x2z))*(amz - x5z) - cos(conj(x2z))*sin(conj(x2x))*(amy - x5y),                                           (cos(conj(x2y))*sin(conj(x2z)) + cos(conj(x2z))*sin(conj(x2x))*sin(conj(x2y)))*(amz - x5z) - (sin(conj(x2y))*sin(conj(x2z)) - cos(conj(x2y))*cos(conj(x2z))*sin(conj(x2x)))*(amx - x5x),   (cos(conj(x2y))*cos(conj(x2z)) - sin(conj(x2x))*sin(conj(x2y))*sin(conj(x2z)))*(amx - x5x) + (cos(conj(x2z))*sin(conj(x2y)) + cos(conj(x2y))*sin(conj(x2x))*sin(conj(x2z)))*(amz - x5z) - cos(conj(x2x))*sin(conj(x2z))*(amy - x5y), 0, 0, 0,                                                                                                    0,  0,                                                                                                   0, - cos(conj(x2y))*sin(conj(x2z)) - cos(conj(x2z))*sin(conj(x2x))*sin(conj(x2y)), -cos(conj(x2x))*cos(conj(x2z)),   cos(conj(x2y))*cos(conj(x2z))*sin(conj(x2x)) - sin(conj(x2y))*sin(conj(x2z)),
				0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                cos(conj(x2x))*(amy - x5y) - cos(conj(x2y))*sin(conj(x2x))*(amz - x5z) + sin(conj(x2x))*sin(conj(x2y))*(amx - x5x),                                                                                                                                           - cos(conj(x2x))*cos(conj(x2y))*(amx - x5x) - cos(conj(x2x))*sin(conj(x2y))*(amz - x5z),                                                                                                                                                                                                                                     0, 0, 0, 0,                                                                                                    0,  0,                                                                                                   0,                                                  cos(conj(x2x))*sin(conj(x2y)),                -sin(conj(x2x)),                                                 -cos(conj(x2x))*cos(conj(x2y)),
				0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                     0, 0, 0, 0,                                                                                                    0,  0,                                                                                                   0,                                                                              0,                              0,                                                                              0,
				0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                     0, 0, 0, 0,                                                                                                    0,  0,                                                                                                   0,                                                                              0,                              0,                                                                              0,
				0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                     0, 0, 0, 0,                                                                                                    0,  0,                                                                                                   0,                                                                              0,                              0,                                                                              0,
				0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                     0, 0, 0, 0,                                                                                                    0,  0,                                                                                                   0,                                                                              0,                              0,                                                                              0,
				0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                     0, 0, 0, 0,                                                                                                    0,  0,                                                                                                   0,                                                                              0,                              0,                                                                              0,
				0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                     0, 0, 0, 0,                                                                                                    0,  0,                                                                                                   0,                                                                              0,                              0,                                                                              0;

			MatrixXd Ut(15,12);
			Ut<<0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				-cos(x2y)/(cos(x2y)*cos(x2y) + sin(x2y)*sin(x2y)), 0, -sin(x2y)/(cos(x2y)*cos(x2y) + sin(x2y)*sin(x2y)), 0, 0, 0, 0, 0, 0, 0, 0, 0,
				-(sin(x2x)*sin(x2y))/(cos(x2x)*cos(x2y)*cos(x2y) + cos(x2x)*sin(x2y)*sin(x2y)), -1, (cos(x2y)*sin(x2x))/(cos(x2x)*cos(x2y)*cos(x2y) + cos(x2x)*sin(x2y)*sin(x2y)), 0, 0, 0, 0, 0, 0, 0, 0, 0,
		    	sin(x2y)/(cos(x2x)*cos(x2y)*cos(x2y) + cos(x2x)*sin(x2y)*sin(x2y)), 0, -cos(x2y)/(cos(x2x)*cos(x2y)*cos(x2y) + cos(x2x)*sin(x2y)*sin(x2y)), 0, 0, 0, 0, 0, 0, 0, 0, 0,
		    	0, 0, 0, -(cos(x2y)*cos(x2z) - sin(x2x)*sin(x2y)*sin(x2z)), cos(x2x)*sin(x2z), -(cos(x2z)*sin(x2y) + cos(x2y)*sin(x2x)*sin(x2z)), 0, 0, 0, 0, 0, 0,
				0, 0, 0, -(cos(x2y)*sin(x2z) + cos(x2z)*sin(x2x)*sin(x2y)), -cos(x2x)*cos(x2z), -(sin(x2y)*sin(x2z) - cos(x2y)*cos(x2z)*sin(x2x)), 0, 0, 0, 0, 0, 0,
		        0, 0, 0, cos(x2x)*sin(x2y), -sin(x2x), -cos(x2x)*cos(x2y), 0, 0, 0, 0, 0, 0,
		        0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0,
		        0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0,
		        0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0,
		        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
		        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
		        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1;


			Ft = I15 + dT*At; // calculate Ft
			Vt = dT*Ut; // calculate Vt

			cov = Ft*cov*Ft.transpose() + Vt*Q*Vt.transpose(); // update covariance

		    Matrix3d R(3,3);
		    Matrix3d G_inv(3,3);
		    G_inv<<  cos(x2y)/(cos(x2y)*cos(x2y) + sin(x2y)*sin(x2y)), 0, sin(x2y)/(cos(x2y)*cos(x2y) + sin(x2y)*sin(x2y)),
		    		(sin(x2x)*sin(x2y))/(cos(x2x)*cos(x2y)*cos(x2y) + cos(x2x)*sin(x2y)*sin(x2y)), 1, -(cos(x2y)*sin(x2x))/(cos(x2x)*cos(x2y)*cos(x2y) + cos(x2x)*sin(x2y)*sin(x2y)),
	        		-sin(x2y)/(cos(x2x)*cos(x2y)*cos(x2y) + cos(x2x)*sin(x2y)*sin(x2y)), 0, cos(x2y)/(cos(x2x)*cos(x2y)*cos(x2y) + cos(x2x)*sin(x2y)*sin(x2y));
	        
			R<< cos(x2y)*cos(x2z) - sin(x2x)*sin(x2y)*sin(x2z), -cos(x2x)*sin(x2z), cos(x2z)*sin(x2y) + cos(x2y)*sin(x2x)*sin(x2z),
				cos(x2y)*sin(x2z) + cos(x2z)*sin(x2x)*sin(x2y),  cos(x2x)*cos(x2z), sin(x2y)*sin(x2z) - cos(x2y)*cos(x2z)*sin(x2x),
			    -cos(x2x)*sin(x2y), sin(x2x), cos(x2x)*cos(x2y);
			MatrixXd wm(3,1);
			wm<<wmx,
				wmy,
				wmz;
			MatrixXd am(3,1);
			am<<amx,
				amy,
				amz;
			MatrixXd wmm(3,1);
			wmm=G_inv*wm;
			MatrixXd a(3,1);
			a=R*am;
			f(0,0)=x(6,0);
			f(1,0)=x(7,0);
			f(2,0)=x(8,0);
			f(3,0)=wmm(0,0);
			f(4,0)=wmm(1,0);
			f(5,0)=wmm(2,0);
			f(6,0)=a(0,0);
			f(7,0)=a(1,0);
			f(8,0)=a(2,0)-9.8;
			for(int i=9;i<15;i++)
				f(i,0)=0;

			x = x + f*dT;
		}
	}
}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //your code for update
    //camera position in the IMU frame = (0, -0.04, -0.02)
    //camera orientation in the IMU frame = Quaternion(0, 0, 1, 0); w x y z, respectively
    //RotationMatrix << -1, 0, 0,
    //  			     0, 1, 0,
    //                   0, 0, -1;
    
    //cur_t = msg->header.stamp.toSec();
    MatrixXd Tic(3,1);
    Tic<<0,
    	 -0.04,
    	 -0.02;
    MatrixXd Twi(3,1);
    MatrixXd Tcw(3,1);
    MatrixXd Pi(3,1);
    MatrixXd Pw(3,1);

    MatrixXd Ct(6,15);
    Ct<<1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    	0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
    	0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,
    	0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,
    	0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
    	0,0,0,0,0,1,0,0,0,0,0,0,0,0,0;

    MatrixXd Wt(6,6);
    Wt<<1,0,0,0,0,0,
    	0,1,0,0,0,0,
    	0,0,1,0,0,0,
    	0,0,0,1,0,0,
    	0,0,0,0,1,0,
    	0,0,0,0,0,1;

    MatrixXd Kt(15,6);
    Kt=cov*Ct.transpose()*(Ct*cov*Ct.transpose()+Wt*Rt*Wt.transpose()).inverse();

    MatrixXd Zt(6,1);
    Quaterniond quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

	Tcw<<
		msg->pose.pose.position.x,
    	msg->pose.pose.position.y,
    	msg->pose.pose.position.z;
    Twi=-quaternion.toRotationMatrix().transpose()*(Rcam.inverse()*Tic+Tcw);

    Matrix3d Rwi(3,3);
    Rwi=(Rcam*quaternion.toRotationMatrix()).inverse();
	double wx = asin(Rwi(2,1)); // phi angle
	double wy = asin(Rwi(2,0)/(-cos(wx))); // theta angle
	double wz = acos(Rwi(1,1)/cos(wx)); // kesi angle

    Zt<<Twi(0,0),																
    	Twi(1,0),
    	Twi(2,0),
    	wx,
    	wy,
    	wz;

	if(!cam_ready)
		{
			for(int i=0;i<6;i++)
				x(i,0)=Zt(i,0);
		}

    MatrixXd miu(6,1);
    miu<<x(0,0),
    	 x(1,0),
    	 x(2,0),
    	 x(3,0),
    	 x(4,0),
    	 x(5,0);

    MatrixXd zta(6,1);
    zta<<
    	Zt(0,0)-x(0,0),
    	Zt(1,0)-x(1,0),
    	Zt(2,0)-x(2,0),
    	wx-x(3,0),
    	wy-x(4,0),
    	wz-x(5,0);

    if(zta(3,0)>3.14)
    	zta(3,0)=zta(3,0)-2*3.14;
    else if(zta(3,0)<-3.14)
    	zta(3,0)=zta(3,0)+2*3.14;

    if(zta(4,0)>3.14)
    	zta(4,0)=zta(4,0)-2*3.14;
    else if(zta(4,0)<-3.14)
    	zta(4,0)=zta(4,0)+2*3.14;

    if(zta(5,0)>3.14)
    	zta(5,0)=zta(5,0)-2*3.14;
    else if(zta(5,0)<-3.14)
    	zta(5,0)=zta(5,0)+2*3.14;

    x=x+Kt*(zta); // update the pose
    cov=cov-Kt*Ct*cov; // update covariance

    Matrix3d Ro(3,3);
	double x2x=x(3,0);
	double x2y=x(4,0);
	double x2z=x(5,0);
	Ro<<cos(x2y)*cos(x2z) - sin(x2x)*sin(x2y)*sin(x2z), -cos(x2x)*sin(x2z), cos(x2z)*sin(x2y) + cos(x2y)*sin(x2x)*sin(x2z),
		cos(x2y)*sin(x2z) + cos(x2z)*sin(x2x)*sin(x2y),  cos(x2x)*cos(x2z), sin(x2y)*sin(x2z) - cos(x2y)*cos(x2z)*sin(x2x),
        -cos(x2x)*sin(x2y),           sin(x2x),                              cos(x2x)*cos(x2y);

	Quaterniond Qu(Ro);

	nav_msgs::Odometry odom_me;
	odom_me.header.stamp = msg->header.stamp;
    odom_me.header.frame_id = "world";
    odom_me.pose.pose.position.x = x(0,0);
    odom_me.pose.pose.position.y = x(1,0);
    odom_me.pose.pose.position.z = x(2,0);
    odom_me.twist.twist.linear.x = x(6,0);
    odom_me.twist.twist.linear.y = x(7,0);
    odom_me.twist.twist.linear.z = x(8,0);
    odom_me.pose.pose.orientation.w = Qu.w();
    odom_me.pose.pose.orientation.x = Qu.x();
    odom_me.pose.pose.orientation.y = Qu.y();
    odom_me.pose.pose.orientation.z = Qu.z();
    odom_pub.publish(odom_me);
    cam_ready=1;
    //cout<<x(0,0)<<" "<<x(1,0)<<" "<<x(2,0)<<endl;
    //cout<<Qu.w()<<" "<<Qu.x()<<" "<<Qu.y()<<" "<<Qu.z()<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    Rcam = Quaterniond(0, 0, -1, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl;
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);   
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6); 
    Rt.topLeftCorner(3, 3) = 0.5 * Rt.topLeftCorner(3, 3);  
    Rt.bottomRightCorner(3, 3) = 0.5 * Rt.bottomRightCorner(3, 3); 
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1); 

    ros::spin();
}
