function s_des = trajectory_generator(t, path, h)
persistent avg_speed;
persistent Px;
persistent Py;
persistent Pz;
persistent Ta;
persistent m;
if nargin > 1 % pre-process can be done here (given waypoints). Pre-define the entire trajectory.
    [m,n]=size(path);
    L=0;
    for i=1:m-1
        L=L+sqrt((path(i+1)-path(i))^2+(path(i+1+m)-path(i+m))^2+(path(i+1+2*m)-path(i+2*m))^2);
    end
    avg_speed=L/25;
    Px=zeros(8*(m-1),1); % Px is from Pi0-Pi7 with i from 1 to m-1
    Py=zeros(8*(m-1),1);
    Pz=zeros(8*(m-1),1);
    Q=zeros(8*(m-1),8*(m-1));
    A=zeros(6*(m-1),8*(m-1));
    Dx=zeros(6*(m-1),1);
    Dy=zeros(6*(m-1),1);
    Dz=zeros(6*(m-1),1);
    T=zeros(m-1,1);
    Ta=zeros(m,1);
    s_des=zeros(13,1);
    
    for i=1:m-1
        T(i)=sqrt((path(i+1)-path(i))^2+(path(i+1+m)-path(i+m))^2+(path(i+1+m*2)-path(i+m*2))^2)/avg_speed;
    end
    Ta(1)=0;
    Ta(2)=T(1);
    for i=3:m
        Ta(i)=Ta(i-1)+T(i-1);
    end
    for i=1:m-1
        for j=1:4
            for k=1:4
                Q((8*(i-1)+4)*8*(m-1) + 8*(i-1)+4 + 8*(m-1)*(j-1) + k)=(3+k)*(2+k)*(1+k)*(k)*(3+j)*(2+j)*(1+j)*(j)/(j+k-1)*T(i)^(j+k-1);
            end
        end
    end
    
    for i=1:m-1
        A(8*(i-1)*6*(m-1) + 6*(i-1) + 1)=1; % calculate initial position at each segment
        for j=1:8 % calculate final position at each segment
            A(8*(i-1)*6*(m-1) + 6*(i-1) + (j-1)*6*(m-1) + 2)=T(i)^(j-1);
        end
        A(6*(m-1) + 3)=1; % only the initial velocity is known for the very first segment
        for j=1:7
            A(8*(i-1)*6*(m-1) + 6*(i-1) + (8-j)*6*(m-1) + 4)=(8-j)*T(i)^(7-j);
        end
        A(2*6*(m-1) + 5)=2; % only the initial acceleration is known
        for j=1:6
            A(8*(i-1)*6*(m-1) + 6*(i-1) + (8-j)*6*(m-1) + 6)=(8-j)*(7-j)*T(i)^(6-j);
        end
    end
    
    for i=1:m-2
        A(8*6*(m-1)*i + 6*(i-1) + 4 + 1*6*(m-1))=-1; % calculate A(j)-A(j+1) = 0
        A(8*6*(m-1)*i + 6*(i-1) + 6 + 2*6*(m-1))=-2;
    end
    
    for i=1:m-1
        Dx(6*(i-1) + 1)=path(1+(i-1));
        Dx(6*(i-1) + 2)=path(2+(i-1));
        Dy(6*(i-1) + 1)=path(1+(i-1)+m);
        Dy(6*(i-1) + 2)=path(2+(i-1)+m);
        Dz(6*(i-1) + 1)=path(1+(i-1)+2*m);
        Dz(6*(i-1) + 2)=path(2+(i-1)+2*m);
    end
    
    Px = quadprog(Q,[],[],[],A,Dx);
    Py = quadprog(Q,[],[],[],A,Dy);
    Pz = quadprog(Q,[],[],[],A,Dz);
else % output desired state along the trajectory here (given time).
    key=0;
    for i=1:m-1
        if(t>=Ta(i) && t<=Ta(i+1))
            key=i;
            t=t-Ta(i);
            break;
        end
    end
    
    key=key-1;
    s_des(1)=Px(8+key*8)*t^7+Px(7+key*8)*t^6+Px(6+key*8)*t^5+Px(5+key*8)*t^4+Px(4+key*8)*t^3+Px(3+key*8)*t^2+Px(2+key*8)*t^1+Px(1+key*8)*t^0;
    s_des(2)=Py(8+key*8)*t^7+Py(7+key*8)*t^6+Py(6+key*8)*t^5+Py(5+key*8)*t^4+Py(4+key*8)*t^3+Py(3+key*8)*t^2+Py(2+key*8)*t^1+Py(1+key*8)*t^0;
    s_des(3)=Pz(8+key*8)*t^7+Pz(7+key*8)*t^6+Pz(6+key*8)*t^5+Pz(5+key*8)*t^4+Pz(4+key*8)*t^3+Pz(3+key*8)*t^2+Pz(2+key*8)*t^1+Pz(1+key*8)*t^0;
    s_des(4)=7*Px(8+key*8)*t^6+6*Px(7+key*8)*t^5+5*Px(6+key*8)*t^4+4*Px(5+key*8)*t^3+3*Px(4+key*8)*t^2+2*Px(3+key*8)*t^1+Px(2+key*8)*t^0;
    s_des(5)=7*Py(8+key*8)*t^6+6*Py(7+key*8)*t^5+5*Py(6+key*8)*t^4+4*Py(5+key*8)*t^3+3*Py(4+key*8)*t^2+2*Py(3+key*8)*t^1+Py(2+key*8)*t^0;
    s_des(6)=7*Pz(8+key*8)*t^6+6*Pz(7+key*8)*t^5+5*Pz(6+key*8)*t^4+4*Pz(5+key*8)*t^3+3*Pz(4+key*8)*t^2+2*Pz(3+key*8)*t^1+Pz(2+key*8)*t^0;
    
    s_des(7:10)=R_to_quaternion(ypr_to_R([0,0,0]));
    s_des = s_des';
end

end


