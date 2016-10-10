% function [F, M] = controller(t, s, s_des)
function [F, M, RMS_v, RMS_p] = controller(t, s, s_des, RMS_v, RMS_p)
global params

m = params.mass;
g = params.grav;
I = params.I;

% You should calculate the output F and M
Kd1=3;
Kp1=5;
Kd2=3;
Kp2=5;
Kd3=10;
Kp3=10;
KpPhi=1000;
KpTheta=1000;
KpYawangle=150;
KdPhi=15;
KdTheta=15;
KdYawangle=15;

r1_des2=Kd1*(s_des(4)-s(4))+Kp1*(s_des(1)-s(1)); % second derivative of desired x
r2_des2=Kd2*(s_des(5)-s(5))+Kp2*(s_des(2)-s(2));
r3_des2=Kd3*(s_des(6)-s(6))+Kp3*(s_des(3)-s(3));

Rot_des = QuatToRot([s_des(7), s_des(8), s_des(9), s_des(10)]');
[phi_des, theta_des, yawangle_des] = RotToRPY_ZXY(Rot_des); % desired angle calculated from quaternion

Rot = QuatToRot([s(7),s(8),s(9),s(10)]');
[phi,theta,yawangle] = RotToRPY_ZXY(Rot);

phi_des = 1/g*(r1_des2*sin(yawangle)-r2_des2*cos(yawangle));
theta_des = 1/g*(r1_des2*cos(yawangle)+ r2_des2*sin(yawangle));

phi1 = s(11);
theta1 = s(12);
yawangle1 = s(13);

phi_des1 = 0;
theta_des1 = 0;
yawangle_des1 = 0;

dYaw = yawangle_des-yawangle;

if(dYaw >= pi)
    dYaw = -( 2*pi - dYaw);
elseif(dYaw <= -pi)
    dYaw = 2*pi + dYaw;
end

phi_des2 = KpPhi*(phi_des-phi)+KdPhi*(phi_des1-phi1); % second derivative of desired angle phi
theta_des2 = KpTheta*(theta_des-theta)+KdTheta*(theta_des1-theta1);
yawangle_des2 = KpYawangle*(dYaw)+KdYawangle*(yawangle_des1-yawangle1);

F = m*(g+r3_des2);

M = I*[phi_des2, theta_des2, yawangle_des2]'+...
    cross([s(11), s(12), s(13)]',I*[s(11), s(12), s(13)]');

times=t/0.01;
RMS_v = sqrt((RMS_v*RMS_v*(times-1)+(s_des(5)-s(5))*(s_des(5)-s(5)))/times);
RMS_p = sqrt((RMS_p*RMS_p*(times-1)+(s_des(2)-s(2))*(s_des(2)-s(2)))/times);
RMS_v
RMS_p

% M = I*[phi_des2, theta_des2, yawangle_des2]';
% M =[0,0,0]';
end
