#ifndef trajectory_h
#define trajectory_h
#include <stdio.h>
#include <math.h>

using namespace Eigen;

/*
 * this function is to get desired states for specific trajectory, just generated, at time dt.
 * input:
 * dT   -> the time
 * hover_pos -> the desired position where you want quadrotor to hover
 * now_vel -> maybe useless
 *
 * output:
 * desired_pos -> desired position at dT
 * desired_vel -> desired velocity at dT
 * desired_acc -> desired acceleration at dT
 * return:
 * true  -> you have alread configured desired states
 * false -> no desired state
 */
bool trajectory_control(const double dT, 
        const Vector3d hover_pos,
        const Vector3d now_vel,
        Vector3d & desired_pos,
        Vector3d & desired_vel,
        Vector3d & desired_acc
        )
{
    //if you don't want to use Eigen, then you can use these arrays
    //or you can delete them and use Eigen
    double hover_p[3], now_v[3], desired_p[3], desired_v[3], desired_a[3];
    hover_p[0] = hover_pos.x();
    hover_p[1] = hover_pos.y();
    hover_p[2] = hover_pos.z();
    now_v[0] = now_vel.x();
    now_v[1] = now_vel.y();
    now_v[2] = now_vel.z();
    //your code // please use coefficients from matlab to get desired states

	int key;
	double px[48]={0,0,0,0.0083,-0.00045287,0.000062706,0,0,
		    0.3,0.1223,-0.0122,-0.0011,0,0.00012193,-0.000016722,0,
		    0.6,0.048,0.0058,0.0003456,0,-0.000034836,0,0,
		    0.9,0.0877,0,-0.0012,0,0.000017418,0,0,
		    1.2,0.0480,-0.0058,0.0035,0,-0.000034836,0,0,
		    1.5,0.1223,0.0122,-0.0083,0,0.00012193,0.000016722,0};
	double py[48]={0,0,0,0,0,0,0,0,
		    0,0,0,0,0,0,0,0,
		    0,0,0,0,0,0,0,0,
		    0,0,0,0,0,0,0,0,
		    0,0,0,0,0,0,0,0,
		    0,0,0,0,0,0,0,0};
	double pz[48]={0,0,0,0,0,0,0,0,
		    0,0,0,0,0,0,0,0,
		    0,0,0,0,0,0,0,0,
		    0,0,0,0,0,0,0,0,
		    0,0,0,0,0,0,0,0,
		    0,0,0,0,0,0,0,0};
	double Ta[7]={0,4.1667,8.3333,12.5,16.6667,20.8333,25.0000};

	key=0;
	double t=0;
	for (int i=0; i<6; i++)
	{
	  if(dT>=Ta[i] && dT<=Ta[i+1])
	  {
	    key=i;
	    t=dT-Ta[i];
	    break;
	  }
	}
	
	desired_p[0]=px[7+key*8]*pow(t,7)+px[6+key*8]*pow(t,6)+px[5+key*8]*pow(t,5)+px[4+key*8]*pow(t,4)+
			px[3+key*8]*pow(t,3)+px[2+key*8]*pow(t,2)+px[1+key*8]*pow(t,1)+px[0+key*8]*pow(t,0);
	desired_p[1]=py[7+key*8]*pow(t,7)+py[6+key*8]*pow(t,6)+py[5+key*8]*pow(t,5)+py[4+key*8]*pow(t,4)+
			py[3+key*8]*pow(t,3)+py[2+key*8]*pow(t,2)+py[1+key*8]*pow(t,1)+py[0+key*8]*pow(t,0);
	desired_p[2]=pz[7+key*8]*pow(t,7)+pz[6+key*8]*pow(t,6)+pz[5+key*8]*pow(t,5)+pz[4+key*8]*pow(t,4)+
			pz[3+key*8]*pow(t,3)+pz[2+key*8]*pow(t,2)+pz[1+key*8]*pow(t,1)+pz[0+key*8]*pow(t,0);
	desired_v[0]=7*px[7+key*8]*pow(t,6)+6*px[6+key*8]*pow(t,5)+5*px[5+key*8]*pow(t,4)+4*px[4+key*8]*pow(t,3)+
			3*px[3+key*8]*pow(t,2)+2*px[2+key*8]*pow(t,1)+1*px[1+key*8]*pow(t,0);
	desired_v[1]=7*py[7+key*8]*pow(t,6)+6*py[6+key*8]*pow(t,5)+5*py[5+key*8]*pow(t,4)+4*py[4+key*8]*pow(t,3)+
			3*py[3+key*8]*pow(t,2)+2*py[2+key*8]*pow(t,1)+1*py[1+key*8]*pow(t,0);
	desired_v[2]=7*pz[7+key*8]*pow(t,6)+6*pz[6+key*8]*pow(t,5)+5*pz[5+key*8]*pow(t,4)+4*pz[4+key*8]*pow(t,3)+
			3*pz[3+key*8]*pow(t,2)+2*pz[2+key*8]*pow(t,1)+1*pz[1+key*8]*pow(t,0);
	if(dT>Ta[6])
	{
	  desired_p[0]=1.8;
	  desired_p[1]=0;
	  desired_p[2]=0;
	  desired_v[2]=0;desired_v[1]=0;desired_v[2]=0;
	  desired_a[0]=0;desired_a[1]=0;desired_a[2]=0;
	}	

    //output
    desired_pos.x() = desired_p[0];
    desired_pos.y() = desired_p[1];
    desired_pos.z() = desired_p[2];
    desired_vel.x() = desired_v[0];
    desired_vel.y() = desired_v[1];
    desired_vel.z() = desired_v[2];
    desired_acc.x() = 0; //desired_a[0];
    desired_acc.y() = 0; //desired_a[1];
    desired_acc.z() = 0; //desired_a[2];

    return true; // if you have got desired states, true.
}
#endif
