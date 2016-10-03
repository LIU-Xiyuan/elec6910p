function [F, M] = controller(t, s, s_des)

global params

m = params.mass;
g = params.grav;
I = params.I;

% You should calculate the output F and M
Kd1=2;
Kp1=3;
Kd2=2;
Kp2=3;
Kd3=10;
Kp3=10;
KpPhi=1500;
KpTheta=1500;
KpYawangle=15;
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
theta_des = 1/g*(r1_des2*cos(yawangle)-r2_des2*sin(yawangle));

phi1 = s(11);
theta1 = s(12);
yawangle1 = s(13);

% phi_des1 = s_des(11); % first derivative of desired angle phi
% theta_des1 = s_des(12);
% yawangle_des1 = s_des(13);
phi_des1 = 0;
theta_des1 = 0;
yawangle_des1 = 0;

phi_des2 = KpPhi*(phi_des-phi)+KdPhi*(phi_des1-phi1); % second derivative of desired angle phi
theta_des2 = KpTheta*(theta_des-theta)+KdTheta*(theta_des1-theta1);
yawangle_des2 = KpYawangle*(yawangle_des-yawangle)+KdYawangle*(yawangle_des1-yawangle1);

F = m*(g+r3_des2);

% M = I*[phi_des2, theta_des2, yawangle_des2]'+...
%     cross([s(11), s(12), s(13)]',I*[s(11), s(12), s(13)]');
M = I*[phi_des2, theta_des2, yawangle_des2]';
end
