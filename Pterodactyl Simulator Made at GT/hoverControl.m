clear all
clc

phiL = 89*pi/180;
phiR = 90*pi/180;
psi = 30*pi/180;

[comX,comY,comZ] = massCenter(phiL,phiR);

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
State = [0;0;-100;-0.05;0.2;0.01;0;0;0;0;0;0];
Ib = [45239.31  -43.96  -91.58;...
      -43.96    991.74  0.90;...
      -91.58    0.90    54968.33];
m = 46.18;
Force = [0;0;0];
Moment = [0;0;0];

size= 20000;
testF = zeros(3,size);
testR = zeros(1,size);
testP = zeros(1,size);
testY = zeros(1,size);

copter = eulerRK4(State,Ib,m,Force,Moment);

cmdAngle = [0;0;0];
kPhi = 5;
kTheta = 5;
kPsi = 5;
gainOuter = [-kPhi 0 0;0 -kTheta 0;0 0 -kPsi];

 for i =1:size
     testR(i) = copter.State(4);
     testP(i) = copter.State(5);
     testY(i) = copter.State(6);
     
     errAngle = gainOuter*(copter.State(4:6)-cmdAngle);
     R = [1 sin(copter.State(4))*tan(copter.State(5)) cos(copter.State(4))*tan(copter.State(5));...
          0 cos(copter.State(4)) -sin(copter.State(4));...
          0 sin(copter.State(4))/cos(copter.State(5)) cos(copter.State(4))/cos(copter.State(5))];
      
     cmdRate = R\errAngle;
     errRate = copter.State(10:12)-cmdRate;
     cmdAcc = [0;0;0];
      
     alpha = -1/(Ib(1,1)*Ib(3,3)-Ib(1,3)^2);
     k1 = 10000;
     k2 = 1000;
     k3 = 10000;
     
     Mx = alpha*alpha*(Ib(1,1)*(-k1*errRate(1)+alpha*Ib(1,3)*copter.State(12)-Ib(3,3)*alpha*copter.State(10)+cmdAcc(1))+Ib(1,3)*(-k3*errRate(3)+Ib(1,3)*alpha*copter.State(10)-Ib(1,1)*alpha*copter.State(12)+cmdAcc(3)));
     My = copter.State(11)-k2*errRate(2)+cmdAcc(2);
     Mz = alpha*alpha*(Ib(1,3)*(-k1*errRate(1)+alpha*Ib(1,3)*copter.State(12)-Ib(3,3)*alpha*copter.State(10)+cmdAcc(1))+Ib(3,3)*(-k3*errRate(3)+Ib(1,3)*alpha*copter.State(10)-Ib(1,1)*alpha*copter.State(12)+cmdAcc(3)));
     %Mx = copter.State(10)-k1*errRate(1)+cmdAcc(1);
     %Mz = copter.State(12)-k3*errRate(3)+cmdAcc(3);
     thrust = A\[Mx;My;Mz];

     copter.Moment = A*thrust;
     copter.Force= thrust(1)*[a1;b1;c1] + [0; 0; -thrust(2)] + thrust(3)*[a2;b2;c2]; %MIDDLE TERM CORRECT?
     testF(:,i) = copter.Force;
     
     copter.State = copter.homebrewRK4();
 end
 plot(testR,'g');
 hold on
 plot(testP);
 plot(testY,'r');
