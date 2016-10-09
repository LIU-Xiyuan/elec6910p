function [F, M] = controller(t, s, s_des)

global params

m = params.mass;
g = params.grav;
I = params.I;

%F = 1.0;
%F = m*g;
%M = [0.0, 0.0, 0.0]'; % You should calculate the output F and M

persistent error_square_x;
persistent error_square_y;
persistent error_square_z;
persistent error_square_x_v;
persistent error_square_y_v;
persistent error_square_z_v;

persistent n;


if isempty(error_square_x)
    error_square_x = 0;
end
if isempty(error_square_y)
    error_square_y = 0;
end
if isempty(error_square_z)
    error_square_z = 0;
end
if isempty(error_square_x_v)
    error_square_x_v = 0;
end
if isempty(error_square_y_v)
    error_square_y_v = 0;
end
if isempty(error_square_z_v)
    error_square_z_v = 0;
end
if isempty(n)
    n = 0;
end

error_square_x=error_square_x+(s(1)-s_des(1))*(s(1)-s_des(1));
error_square_y=error_square_y+(s(2)-s_des(2))*(s(2)-s_des(2));
error_square_z=error_square_z+(s(3)-s_des(3))*(s(3)-s_des(3));

error_square_x_v=error_square_x_v+(s(4)-s_des(4))*(s(4)-s_des(4));
error_square_y_v=error_square_y_v+(s(5)-s_des(5))*(s(5)-s_des(5));
error_square_z_v=error_square_z_v+(s(6)-s_des(6))*(s(6)-s_des(6));

n=n+1;
rms_x=sqrt(error_square_x./n);
rms_y=sqrt(error_square_y./n);
rms_z=sqrt(error_square_z./n);
rms_x_v=sqrt(error_square_x_v./n);
rms_y_v=sqrt(error_square_y_v./n);
rms_z_v=sqrt(error_square_z_v./n);

if abs(t-25)<0.001
   rms_x
   rms_y
   rms_z
   rms_x_v
   rms_y_v
   rms_z_v
end

Kd1=2;
Kd2=2;
Kd3=10;
Kp1=3;
Kp2=3;
Kp3=10;
Kpphi=1500;
Kptheta=1500;
Kppsi=15;

Kdphi=15;
Kdtheta=15;
Kdpsi=15; 
r1c_ddo=Kd1*(s_des(4)-s(4))+Kp1*(s_des(1)-s(1));
r2c_ddo=Kd2*(s_des(5)-s(5))+Kp2*(s_des(2)-s(2));
r3c_ddo=Kd3*(s_des(6)-s(6))+Kp3*(s_des(3)-s(3));

F=m*(g+r3c_ddo);

q=s(7:10);
R=quaternion_to_R(q);
[phi,theta,psi]=RotToRPY_ZXY(R);

phi_c=(1/g)*(r1c_ddo*sin(psi)-r2c_ddo*cos(psi));
theta_c=(1/g)*(r1c_ddo*cos(psi)+r2c_ddo*sin(psi));

q_c=s_des(7:10);
R_c=quaternion_to_R(q_c);
[dummy1,dummy2,psi_c]=RotToRPY_ZXY(R_c);

p = s(11);
q = s(12);
r = s(13);

p_des = s_des(11);
q_des = s_des(12);
r_des = s_des(13);



line1=Kpphi*(phi_c-phi)+ Kdphi * (p_des - p);
line2=Kptheta*(theta_c-theta)+Kdtheta * (q_des - q);

psi_diff=psi_c-psi;

if(psi_diff>pi)
    psi_diff=psi_diff-2*pi;
end
if(psi_diff<(-pi))
    psi_diff=psi_diff+2*pi;
end


line3=Kppsi*(psi_diff)+Kdpsi * (r_des-r);

acc=[line1;line2;line3];
M=I*acc;
end
