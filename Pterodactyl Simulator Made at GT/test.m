function [stability,err] = test(phiL,phiR,psi,k)
%{
phiL = 85*pi/180;
phiR = 100*pi/180;
psi = 65*pi/180;

%}

psi = psi*pi/180;
phiL = phiL*pi/180;
phiR = phiR*pi/180;
%}
k = 30;
d = 60;
t = 7.5;

m = 15;

[comX,comY,comZ,Ix,Iy,Iz] = massCenter(m,phiL,phiR,psi);
Ix %= abs(Ix)
Iy %= abs(Iy)
Iz %= abs(Iz)
%{
Ix = 63;
Iy =6.3;
Iz = 63;
%}
hingeY = 1.5;
hinge2Prop = 0.75;

% Left wing thrust coefficients
a1 = sin(psi)*sin(phiL);
b1 = sin(psi)*cos(psi) - sin(psi)*cos(psi)*cos(phiL);
c1 = -(cos(psi)^2 + cos(phiL)*sin(psi)^2);

% Right wing thrust coefficients
a2 = sin(psi)*sin(phiR);
b2 = sin(psi)*cos(psi)*cos(phiR)-sin(psi)*cos(psi);
c2 = -(cos(psi)^2 + cos(phiR)*sin(psi)^2);



tmp =       [0 0 1 0;0 1 0 0;-1 0 0 0]*[1 0 0 -comX;0 1 0 -comY;0 0 1 -comZ;0 0 0 1]*...
            [1 0 0 0;0 1 0 -hingeY; 0 0 1 0; 0 0 0 1]*...
            [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0; 0 0 1 0;0 0 0 1]*...
            [1 0 0 0;0 cos(-phiL) -sin(-phiL) 0;0 sin(-phiL) cos(-phiL) 0;0 0 0 1]*...
            [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0;0 0 1 0;0 0 0 1]*...
            [1 0 0 0;0 1 0 -hinge2Prop;0 0 1 0;0 0 0 1];
 position1 = tmp(1:3,4);
 orientation1 = tmp(1:3,1);
 tmp =  [0 0 1 0;0 1 0 0;-1 0 0 0]*[1 0 0 -comX;0 1 0 -comY;0 0 1 comZ;0 0 0 1]*...
            [1 0 0 0;0 1 0 hingeY; 0 0 1 0; 0 0 0 1]*...
            [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0; 0 0 1 0;0 0 0 1]*...
            [1 0 0 0;0 cos(phiR) -sin(phiR) 0;0 sin(phiR) cos(phiR) 0;0 0 0 1]*...
            [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0;0 0 1 0;0 0 0 1]*...
            [1 0 0 0;0 1 0 hinge2Prop;0 0 1 0;0 0 0 1];
orientation3 = tmp(1:3,1);
position3 = tmp(1:3,4);
position2 = [comZ;-comY;-comX];% DOUBLE CHECK

% A matrix converts thrusts to moments        
%{
I believe this one to be incorrect with regards to thruster 2
A = [(-position1(3)*b1+position1(2)*c1) 0 (-position3(3)*b2+position3(2)*c2);...
     (position1(3)*a1-position1(1)*c1) position2(3) position3(3)*a2-position3(1)*c2;...
     (position1(1)*b1-a1*position1(2)) -position2(2) (-a2*position3(2)+position3(1)*b2)]
 %}
 A = [(-position1(3)*b1+position1(2)*c1) -position2(2) (-position3(3)*b2+position3(2)*c2);...
     (position1(3)*a1-position1(1)*c1) -position2(1) position3(3)*a2-position3(1)*c2;...
     (position1(1)*b1-a1*position1(2)) 0 (-a2*position3(2)+position3(1)*b2)];

 % Simulator setup
 a = random('unif',0,200)/100 -1;
 b = random('unif',0,200)/100 -1;
 c = random('unif',0,200)/100 -1;
 d = random('unif',0,200)/100 -1;
 e = random('unif',0,200)/100 -1;
 f = random('unif',0,200)/100 -1;

State = [0;0;-100;a/2;b/20;c/2;0;0;0;-a;-b;-c];
%State = [0;0;-100;0;0;0.0;0;0;0;0;0;0];

Ib = [Ix 0 0;0 Iy 0;0 0 Iz];

Force = [0;0;0];
Moment = [0;0;0];

size= 10000;

testR = zeros(1,size);
testP = zeros(1,size);
testY = zeros(1,size);
testZ = zeros(1,size);
shit1 = zeros(1,size);
shit2 = zeros(1,size);
shit3 = zeros(1,size);
shit4 = zeros(1,size);
shit = zeros(1,size);


copter = eulerRK4(State,Ib,m,Force,Moment);
%a = [0 1 0 0 0 0;-.60 -.40 0 0 0 0;0 0 0 1 0 0;0 0 -.60 -.40 0 0; 0 0 0 0 0 1;0 0 0 0 -.60 -.40]

a = [0 1 0 0 0 0;-3.24 -2*3.60 0 0 0 0;0 0 0 1 0 0;0 0 -3.24 -2*3.6 0 0; 0 0 0 0 0 1;0 0 0 0 -3.24 -2*3.60];
A
%A = [-.02*orientation1,[0;0;0],.02*orientation3] + A;
 for i =1:size
     testR(i) = copter.State(4);
     testP(i) = copter.State(5);
     testY(i) = copter.State(6);
     testZ(i) = -copter.State(3);

     x = [copter.State(4);copter.State(10);copter.State(5);copter.State(11);copter.State(6);copter.State(12)];
     Or = copter.State(4:6);
     body2World = [cos(Or(1))*cos(Or(2)) cos(Or(1))*sin(Or(2))*sin(Or(3))-sin(Or(1))*cos(Or(3)) cos(Or(1))*sin(Or(2))*cos(Or(3))+sin(Or(1))*sin(Or(3));...
                       sin(Or(1))*cos(Or(2)) sin(Or(1))*sin(Or(2))*sin(Or(3))+cos(Or(1))*cos(Or(3)) sin(Or(1))*sin(Or(2))*cos(Or(3))-cos(Or(1))*sin(Or(3));...
                       -sin(Or(2)) cos(Or(2))*sin(Or(3)) cos(Or(2))*cos(Or(3))];
     f = a*x;
     
     Mx = f(2)*Ix/1;
     My = f(4)*Iy/1;
     Mz = f(6)*Iz/1;
     
     thrust = A\[Mx;My;Mz];
     mx = max(abs(thrust));

     shit1(i) = thrust(1);
     shit2(i) = thrust(2);
     shit3(i) = thrust(3);
     shit4(i) = f(4);
     %{
     if thrust(1) >t
         thrust(1) = t;
     elseif thrust(1) <-t
         thrust(1) = -t;
     end
     if thrust(2) >2*t
         thrust(2) = 2*t;
     elseif thrust(2) <-2*t
         thrust(2) = -2*t;
     end
     if thrust(3) >t
         thrust(3) = t;
     elseif thrust(3) <-t
         thrust(3) = -t;
     end
     %}

     if mod(i, 10) == 1
          copter.Moment = A*thrust;%+.5*thrust(1)*orientation1 + 0*thrust(2)*[0;0;-1] + -.5*thrust(3)*orientation3;
          copter.Force = thrust(1)*[a1;b1;c1] + [0; 0; -thrust(2)] + thrust(3)*[a2;b2;c2]; %MIDDLE TERM CORRECT?
          %tmp = body2World*copter.Force;
     end
     %shit1(i) = tmp(1);
     %shit2(i) = tmp(2);
     %shit3(i) = tmp(3);
     %shit4(i) = norm(tmp);
     %{
     shit1(i) = copter.Force(1);
     shit2(i) = copter.Force(2);
     shit3(i) = copter.Force(3);
     shit(i) = norm([shit1(i);shit2(i);shit3(i)]);
     %}
     copter.State = copter.homebrewRK4();
     

 end
 err = norm(copter.State(4:6));
stability = (err<0.005);



%plot(testZ);
%figure(2)
%plot(shit);

 %figure(iter)

 plot(testR*180/pi,'r');
 hold on
 plot(testP*180/pi,'g');
 plot(testY*180/pi,'b');
 %{
 figure(2)
 plot(shit1,'r');
 hold on
 plot(shit2,'g');
 plot(shit3,'b');
 
 figure(3)
 plot(shit4)
 
 body2World
 %}
 figure(2)
 plot(shit1,'r');
hold on
plot(shit2,'g');
plot(shit3,'b');
figure(3)
plot(shit4,'k');
 
 %{
figure(2)
plot(testZ);
 %}
end