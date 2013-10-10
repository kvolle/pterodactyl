clear all
clc

phiL = 89*pi/180;
phiR = 90*pi/180;
psi = 30*pi/180;

[comX,comY,comZ,Ix,Iy,Iz] = massCenter(phiL,phiR);

hingeY = 1.75;
hinge2Prop = 0.5;

% Left wing thrust coefficients
a1 = -sin(psi)*sin(phiL);
b1 = sin(psi)*cos(psi) - sin(psi)*cos(psi)*cos(phiL);
c1 = -(cos(psi)^2 + cos(phiL)*sin(psi)^2);
%{
Have to account for change in orientation. Old x is now -z, rotated about y
  The old parameters are kept here for left motor for posterity
a1 = cos(psi)^2 + cos(phiL)*sin(psi)^2;
b1 = sin(psi)*cos(psi) - sin(psi)*cos(psi)*cos(phiL);
c1 = -sin(psi)*sin(phiL);
%}

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
State = [0;0;-100;0;0;0;0;0;0;0;0;0];

m = 46.18;
Force = [0;0;0];
Moment = [0;0;0];

size= 5000;
testF = zeros(1,size);
testH = zeros(1,size);
testV = zeros(1,size);

thrust = zeros(4,1);

copter = eulerRK4(State,Ib,m,Force,Moment);

kPos = 10;
kVel = 10;

%Ix = Ib(1,1);
%Iy = Ib(2,2);
%Iz = Ib(3,3);
 for i =1:size
     testF = sum(thrust);
     testH = -copter.State(3);
     testV = -copter.State(9);

     %x1 is height
     %x2 is velocity in world Z
     
     %This is with forces in world coord
     [x1dot;x2dot] = [0 1;0 0]*[copter.State(3);copter.State(9)]+[0 0 0 0 0;1 1 1 1]*thrust;

     copter.Moment = A*[thrust(1);thrust(2)+thrust(3); thrust(4)];
     copter.Force= thrust(1)*[a1;b1;c1] + [0; 0; -thrust(2)] + thrust(3)*[a2;b2;c2]; %MIDDLE TERM CORRECT?
     testF(:,i) = copter.Force;
     
     copter.State = copter.homebrewRK4();
 end
 plot(testR*180/pi,'g.');
 hold on
 plot(testP*180/pi,'.');
 plot(testY*180/pi,'r.');
