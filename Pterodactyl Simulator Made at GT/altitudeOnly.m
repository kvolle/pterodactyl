function [stability,err] = altitudeOnly()

phiL = 90*pi/180;
phiR = 10*pi/180;
psi = 30*pi/180;

m = 10;
[comX,comY,comZ,Ix,Iy,Iz] = massCenter(m,phiL,phiR);

hingeY = 1.5;
hinge2Prop = 0.75;

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
 a = random('unif',0,200)/100 -1;
 b = random('unif',0,200)/100 -1;
 c = random('unif',0,200)/100 -1;
State = [0;0;-100;a;b;c;0;0;0;0;0;0];
%State = [0;0;-100;0;0;0.0;0;0;0;0;0;0];

Ib = [Ix 0 0;0 Iy 0;0 0 Iz];

Force = [0;0;0];
Moment = [0;0;0];

size= 15000;

testR = zeros(1,size);
testP = zeros(1,size);
testY = zeros(1,size);


copter = eulerRK4(State,Ib,m,Force,Moment);

cmdAngle = [0;0;0];
kPhi = 15;
kTheta = 150;
kPsi = 15;
gainOuter = [-kPhi 0 0;0 -kTheta 0;0 0 -kPsi];

goalH = -100;
goalV = 0;
 for i =1:size
     testR(i) = copter.State(4);
     testP(i) = copter.State(5);
     testY(i) = copter.State(6);
    % testH(i) = -copter.State(3);
     
     errAngle = gainOuter*(copter.State(4:6)-cmdAngle);
     R = [1 sin(copter.State(4))*tan(copter.State(5)) cos(copter.State(4))*tan(copter.State(5));...
          0 cos(copter.State(4)) -sin(copter.State(4));...
          0 sin(copter.State(4))/cos(copter.State(5)) cos(copter.State(4))/cos(copter.State(5))];
     
     cmdRate = R\errAngle;
     errRate = copter.State(10:12)-cmdRate;
     cmdAcc = [0;0;0];
      
     
     k1 = 2.25;%.75;
     k2 = 2;%.750;
     k3 = 2.25;%.75;
     
     k1 = 15000;
     k2 = 150;
     k3 = 3500;

     
     Mx = -k1*errRate(1);
     My = -k2*errRate(2);
     Mz = -k3*errRate(3);
     
     thrust = A\[Mx;My;Mz];
     
     for j =1:3
         if thrust(j) >33
             thrust(j) = 33;
         elseif thrust(j) <-33
             thrust(j) = -33;
         end
     end
     %}
     %plot(i/1000,thrust(1),'r.');
     %hold on
     %plot(i/1000,thrust(2),'g.');
     %plot(i/1000,thrust(3),'b.');
%     tmp = norm(thrust);
%     thrust(1) = 100*thrust(1)/tmp;
%     thrust(2) = 100*thrust(2)/tmp;
%     thrust(3) = 100*thrust(3)/tmp;
     %thrust = (10/norm(thrust))*thrust;
     
     copter.Moment = A*thrust;
     copter.Force= thrust(1)*[a1;b1;c1] + [0; 0; -thrust(2)] + thrust(3)*[a2;b2;c2]; %MIDDLE TERM CORRECT?
     %testF(:,i) = copter.Force;
    % f(i) = norm(thrust);
     copter.State = copter.homebrewRK4();
 end
 err = norm(copter.State(4:6));
 stability = err< 0.001;
end
 %{
 hold off
 plot(testR*180/pi,'g.');
 hold on
 plot(testP*180/pi,'.');
 plot(testY*180/pi,'r.');

 %}