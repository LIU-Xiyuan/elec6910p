function [F, M] = controller(t, s, s_des)

global params

m = params.mass;
g = params.grav;
I = params.I;

% You should calculate the output F and M
Kd1=10;
Kp1=10;
Kd2=10;
Kp2=10;
Kd3=15;
Kp3=30;
KpPhi=900;
KpTheta=900;
KpYawangle=900;
KdPhi=50;
KdTheta=50;
KdYawangle=50;

r1_des2=Kd1*(s_des(4)-s(4))+Kp1*(s_des(1)-s(1)); % second derivative of desired x
r2_des2=Kd2*(s_des(5)-s(5))+Kp2*(s_des(2)-s(2));
r3_des2=Kd3*(s_des(6)-s(6))+Kp3*(s_des(3)-s(3));

Rot_des = QuatToRot([s_des(7), s_des(8), s_des(9), s_des(10)]');
[phi_des, theta_des, yawangle_des] = RotToRPY_ZXY(Rot_des); % desired angle calculated from quaternion
% yawangle_des=0.0;
% yawangle_des = yawangle_des + pi;
Rot = QuatToRot([s(7),s(8),s(9),s(10)]');
[phi,theta,yawangle] = RotToRPY_ZXY(Rot);
% yawangle
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

% yawangle_des
% yawangle

dYaw = yawangle_des-yawangle;

% dYaw
if(dYaw >= pi)
    dYaw =  -( 2*pi - dYaw);
elseif(dYaw <= -pi)
    dYaw =  2 * pi + dYaw;
end

dYaw
% if(dYaw<-pi)
%     dYaw=min(abs(dYaw),2*pi-abs(dYaw));
% elseif(dYaw>=pi)
%     dYaw=-min(abs(dYaw),2*pi-abs(dYaw));
% end

phi_des2 = KpPhi*(phi_des-phi)+KdPhi*(phi_des1-phi1); % second derivative of desired angle phi
theta_des2 = KpTheta*(theta_des-theta)+KdTheta*(theta_des1-theta1);
yawangle_des2 = KpYawangle*(dYaw)+KdYawangle*(yawangle_des1-yawangle1);

F = m*(g+r3_des2);

% M = I*[phi_des2, theta_des2, yawangle_des2]'+...
%     cross([s(11), s(12), s(13)]',I*[s(11), s(12), s(13)]');
M = I*[phi_des2, theta_des2, yawangle_des2]';
% M =[0,0,0]';
end
