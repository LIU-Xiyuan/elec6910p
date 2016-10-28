#ifndef control_funtion_h
#define control_funtion_h

#include <math.h>

/*please fill this controller function
 * input:
 * des_pos -> desired position
 * des_vel -> desired velocity
 * des_acc -> desired acceleration
 * des_yaw -> desired yaw angle
 * now_pos -> now psition
 * now_vel -> body velocity
 * Kp      -> P gain for position loop
 * Kd      -> P gain for velocity loop
 * Mass    -> quality of the quadrotor
 *
 
 * output:
 * rpy -> target attitude for autopilot
 * target_thrust     -> target thrust of the quadrotor
 * */
void SO3Control_function( const double des_pos[3],
                          const double des_vel[3],
                          const double des_acc[3],
                          const double des_yaw,
                          const double now_pos[3],
                          const double now_vel[3],
                          const double now_yaw,
                          const double Kp[3],
                          const double Kd[3],
                          const double Mass,
                          const double Gravity,
                          double rpy[3],
                          double &target_thrust
                        )
{
 //       std::cout << "my control function " << std::endl;

   	target_thrust=Mass*(Gravity+Kd[2]*(des_vel[2]-now_vel[2])+Kp[2]*(des_pos[2]-now_pos[2]));
	double rdd_des = Kd[0]*(des_vel[0]-now_vel[0])+Kp[0]*(des_pos[0]-now_pos[0]);
	double pdd_des = Kd[1]*(des_vel[1]-now_vel[1])+Kp[1]*(des_pos[1]-now_pos[1]);
	rpy[0]=1/Gravity*(rdd_des*sin(des_yaw)-pdd_des*cos(des_yaw));
	rpy[1]=1/Gravity*(rdd_des*cos(des_yaw)+pdd_des*sin(des_yaw));
	rpy[2]=des_yaw;
      
      // std::cout << "ROLL:   "<<rpy[0]<<std::endl;
      //  std::cout<< "PITCH:   "<<rpy[1]<<std::endl;
//        std::cout << "YAW:   "<<rpy[2]<<std::endl;
}
#endif
