clear all
clc
phiL = 100*pi/180;
phiR = 105*pi/180;
psi = 10*pi/180;

a=random('unif',0,200)/100 - 1;
b=random('unif',0,200)/100 - 1;
c=random('unif',0,200)/100 - 1;
d=random('unif',0,200)/100 - 1;

State = [0;0;0;cos(a/2);sin(a/2)*b/norm([b c d]);sin(a/2)*c/norm([b c d]);sin(a/2)*d/norm([b c d]);0;0;0;0;0;0];
State(4:7) = [cos(-pi/4);sin(-pi/4);0;0];
m = 15;
Force = [0;0;0];
Moment = [0;0;0];

[comX,comY,comZ,Ix,Iy,Iz] = massCenter(m,phiL,phiR,psi);
Ib = [Ix 0 0;0 Iy 0;0 0 Iz];

copter = quatRK4(State,Ib,m,Force,Moment);

n = 5000;

test=zeros(1,n);

a = [0 1;-7 -0.1];
qOld = 2*atan2(norm(copter.State(5:7)),copter.State(4));
for i=1:n
    i
    test(i) = 2*atan2(norm(copter.State(5:7)),copter.State(4));
    x = [test(i);1000*(test(i)-qOld)];
    f = a*x;
    
    axis = copter.State(5:7)/norm(copter.State(5:7));
    size(Ib);
    size(axis);
    inertia = dot(axis,Ib*axis);
    
    worldMom = f(2)*inertia*axis;
    
    q = copter.State(4:7);
   R = [q(1)^2+q(2)^2-q(3)^2-q(4)^2 2*(q(2)*q(3)-q(1)*q(4)) 2*(q(2)*q(4)+q(1)*q(3));...
        2*(q(2)*q(3)+q(1)*q(4)) q(1)^2-q(2)^2+q(3)^2-q(4)^2 2*(q(3)*q(4)-q(1)*q(2));...
        2*(q(2)*q(4)-q(1)*q(3)) 2*(q(3)*q(4)-q(1)*q(2)) q(1)^2-q(2)^2-q(3)^2+q(4)^2];
     bodyMom = R\worldMom;
     
     copter.Moment = bodyMom;

     copter.State = copter.homebrewRK4();
end

plot(test*180/pi);