function [stability,err] = test(k,d,t)

phiL = 15*pi/180;
phiR = 75*pi/180;
psi = 45*pi/180;

%k = 15;
%d = 17;

m = 15;

[comX,comY,comZ,Ix,Iy,Iz] = massCenter(m,phiL,phiR);


hingeY = 1.5;
hinge2Prop = 0.75;

% Left wing thrust coefficients
a1 = -sin(psi)*sin(phiL);
b1 = sin(psi)*cos(psi) - sin(psi)*cos(psi)*cos(phiL);
c1 = -(cos(psi)^2 + cos(phiL)*sin(psi)^2);

% Right wing thrust coefficients
a2 = -sin(psi)*sin(phiR);
b2 = sin(psi)*cos(psi)*cos(phiR)-sin(psi)*cos(psi);
c2 = -(cos(psi)^2 + cos(phiR)*sin(psi)^2);

position1 = [0 0 1 0;0 1 0 0;-1 0 0 0]*[1 0 0 -comX;0 1 0 -comY;0 0 1 -comZ;0 0 0 1]*...
            [1 0 0 0;0 1 0 -hingeY; 0 0 1 0; 0 0 0 1]*...
            [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0; 0 0 1 0;0 0 0 1]*...
            [1 0 0 0;0 cos(-phiL) -sin(-phiL) 0;0 sin(-phiL) cos(-phiL) 0;0 0 0 1]*...
            [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0;0 0 1 0;0 0 0 1]*...
            [0;-hinge2Prop;0;1];

position2 = [-comZ;-comY;comX];% DOUBLE CHECK

position3 = [0 0 1 0;0 1 0 0;-1 0 0 0]*[1 0 0 -comX;0 1 0 -comY;0 0 1 -comZ;0 0 0 1]*...
            [1 0 0 0;0 1 0 hingeY; 0 0 1 0; 0 0 0 1]*...
            [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0; 0 0 1 0;0 0 0 1]*...
            [1 0 0 0;0 cos(phiR) -sin(phiR) 0;0 sin(phiR) cos(phiR) 0;0 0 0 1]*...
            [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0;0 0 1 0;0 0 0 1]*...
            [0;hinge2Prop;0;1];
            
% A matrix converts thrusts to moments        
A = [(-position1(3)*b1+position1(2)*c1) 0 (-position3(3)*b2+position3(2)*c2);...
     (position1(3)*a1-position1(1)*c1) position2(3) position3(3)*a2-position3(1)*c2;...
     (position1(1)*b1-a1*position1(2)) -position2(2) (-a2*position3(2)+position3(1)*b2)];


 % Simulator setup
 a = random('unif',0,200)/100 -1;
 b = random('unif',0,200)/100 -1;
 c = random('unif',0,200)/100 -1;

State = [0;0;-100;a;b;c;0;0;0;0;0;0];
%State = [0;0;-100;0;0;0.0;0;0;0;0;0;0];

Ib = [Ix 0 0;0 Iy 0;0 0 Iz];

Force = [0;0;0];
Moment = [0;0;0];

size= 10000;

testR = zeros(1,size);
testP = zeros(1,size);
testY = zeros(1,size);


copter = eulerRK4(State,Ib,m,Force,Moment);


 for i =1:size
     testR(i) = copter.State(4);
     testP(i) = copter.State(5);
     testY(i) = copter.State(6);

     x = [copter.State(4);copter.State(10);copter.State(5);copter.State(11);copter.State(6);copter.State(12)];
     
     f = [0 1 0 0 0 0;-k -d 0 0 0 0;0 0 0 1 0 0;0 0 -k -d 0 0; 0 0 0 0 0 1;0 0 0 0 -k -d]*x;
%      f = [0 1 0 0 0 0;-5 -6 0 0 0 0;0 0 0 1 0 0;0 0 -5 -6 0 0; 0 0 0 0 0 1;0 0 0 0 -5 -6]*x;
     
     Mx = f(2)*Ix;
     My = f(4)*Iy;
     Mz = f(6)*Iz;
     thrust = A\[Mx;My;Mz];
     
     for j =1:3
         if thrust(j) >t
             thrust(j) = t;
         elseif thrust(j) <-t
             thrust(j) = -t;
         end
     end
     %}
     %plot(i/1000,thrust(1),'r.');
     %hold on
     %plot(i/1000,thrust(2),'g.');
     %plot(i/1000,thrust(3),'b.');

    

     %copter.Moment = [Mx;My;Mz];
     if mod(i, 10) == 0
          copter.Moment = A*thrust;
         copter.Force= thrust(1)*[a1;b1;c1] + [0; 0; -thrust(2)] + thrust(3)*[a2;b2;c2]; %MIDDLE TERM CORRECT?
     end
     copter.State = copter.homebrewRK4();
 end
 err = norm(copter.State(4:6));
stability = (err<0.005);

%{
 figure(iter)

 plot(testR*180/pi,'r');
 hold on
 plot(testP*180/pi,'g');
 plot(testY*180/pi,'b');

 %}
end