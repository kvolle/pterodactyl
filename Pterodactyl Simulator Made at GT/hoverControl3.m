clear all
clc

phiL = 0*pi/180;
phiR = 0*pi/180;
psi = 15*pi/180;

[comX,comY,comZ] = massCenter(phiL,phiR);

hingeY = 1.75;
hinge2Prop = 0.5;

% Left wing thrust coefficients
a1 = cos(psi)^2 + cos(phiL)*sin(psi)^2;
b1 = sin(psi)*cos(psi) - sin(psi)*cos(psi)*cos(phiL);
c1 = -sin(psi)*sin(phiL);
%{
Have to account for change in orientation. Old x is now -z, rotated about y
  The old parameters are kept here for left motor for posterity
a1 = cos(psi)^2 + cos(phiL)*sin(psi)^2;
b1 = sin(psi)*cos(psi) - sin(psi)*cos(psi)*cos(phiL);
c1 = -sin(psi)*sin(phiL);
%}

% Right wing thrust coefficients
a2 = cos(psi)^2 + cos(phiR)*sin(psi)^2;
b2 = sin(psi)*cos(psi)*cos(phiR)-sin(psi)*cos(psi);
c2 = -sin(psi)*sin(phiR);

position1 = [1 0 0 0;0 1 0 0;0 0 1 0]*[1 0 0 comX;0 1 0 comY;0 0 1 comZ;0 0 0 1]*...% DOUBLE CHECK
            [1 0 0 0;0 1 0 -hingeY; 0 0 1 0; 0 0 0 1]*...
            [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0; 0 0 1 0;0 0 0 1]*...
            [1 0 0 0;0 cos(-phiL) -sin(-phiL) 0;0 sin(-phiL) cos(-phiL) 0;0 0 0 1]*...
            [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0;0 0 1 0;0 0 0 1]*...
            [0;-hinge2Prop;0;1];

position2 = [comX;comY;comZ];% DOUBLE CHECK

position3 = [1 0 0 0;0 1 0 0;0 0 1 0]*[1 0 0 comX;0 1 0 comY;0 0 1 comZ;0 0 0 1]*...%DOUBLE CHECK
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
State = [0;0;-100;cos(pi/4);0;sin(pi/4);0;0;0;0;0;0;0];
Ib = [45239.31  -43.96  -91.58;...
      -43.96    991.74  0.90;...
      -91.58    0.90    54968.33];
m = 46.18;
Force = [0;0;0];
Moment = [0;0;0];
size= 2500;
testZ = zeros(1,size);
testY = zeros(1,size);
testX = zeros(1,size);
testF = zeros(3,size);

copter = quatRK4(State,Ib,m,Force,Moment);
 for i =1:size
     testX(i) = copter.State(1);
     testY(i) = copter.State(2);
     testZ(i) = -copter.State(3);
     copter.Force= [a1*13.09;b1*13.09;c1*13.09] + [20; 0; 0] + [a2*13.09;b2*13.09;c2*13.09];
     %testF(:,i) = copter.Force;
     
     copter.State = copter.homebrewRK4();
 end
 plot(testZ,'g.');
 hold on
 axis equal
 %axis([-0.1 0.1 -0.1 0.1])
 %figure(2)
 %plot(testV);
 