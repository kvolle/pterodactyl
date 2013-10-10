clear all
clc

phiL = 9*pi/180;
phiR = 10*pi/180;
psi = 15*pi/180;

[comX,comY,comZ] = massCenter(phiL,phiR);

hingeY = 1.75;
hinge2Prop = 0.5;

% Simulator setup
State = [0;0;-100;cos(pi/4);0;sin(pi/4);0;0;0;0;0.1;0;0];
Ib = [45239.31  -43.96  -91.58;...
      -43.96    991.74  0.90;...
      -91.58    0.90    54968.33];
m = 46.18;
Force = [0;0;0];
Moment = [0;0;0];

ball = quatRK4(State,Ib,m,Force,Moment);


% Left wing thrust coefficients
a1 = cos(psi)^2 + cos(phiL)*sin(psi)^2;
b1 = sin(psi)*cos(psi) - sin(psi)*cos(psi)*cos(phiL);
c1 = -sin(psi)*sin(phiL);

% Right wing thrust coefficients
a2 = cos(psi)^2 + cos(phiR)*sin(psi)^2;
b2 = sin(psi)*cos(psi)*cos(phiR)-sin(psi)*cos(psi);
c2 = -sin(psi)*sin(phiR);

position1 = [1 0 0 0;0 1 0 0;0 0 1 0]*[1 0 0 -comX;0 1 0 -comY;0 0 1 -comZ;0 0 0 1]*...
            [1 0 0 0;0 1 0 -hingeY; 0 0 1 0; 0 0 0 1]*...
            [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0; 0 0 1 0;0 0 0 1]*...
            [1 0 0 0;0 cos(-phiL) -sin(-phiL) 0;0 sin(-phiL) cos(-phiL) 0;0 0 0 1]*...
            [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0;0 0 1 0;0 0 0 1]*...
            [0;-hinge2Prop;0;1];

position2 = [-comX;-comY;-comZ];

position3 = [1 0 0 0;0 1 0 0;0 0 1 0]*[1 0 0 -comX;0 1 0 -comY;0 0 1 -comZ;0 0 0 1]*...
            [1 0 0 0;0 1 0 hingeY; 0 0 1 0; 0 0 0 1]*...
            [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0; 0 0 1 0;0 0 0 1]*...
            [1 0 0 0;0 cos(phiR) -sin(phiR) 0;0 sin(phiR) cos(phiR) 0;0 0 0 1]*...
            [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0;0 0 1 0;0 0 0 1]*...
            [0;hinge2Prop;0;1];
            
A = [(-position1(3)*b1+position1(2)*c1) 0 (-position3(3)*b2+position3(2)*c2);...
     (position1(3)*a1-position1(1)*c1) position2(3) position3(3)*a2-position3(1)*c2;...
     (position1(1)*b1-a1*position1(2)) -position2(2) (-a2*position3(2)+position3(1)*b2)];
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% ~~~~~~~~~~~~~~~~~ Redefine Axes So Z-axis Remains Down ~~~~~~~~~~~~~~~~~~
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 
 for i = 1:2500
 A2 = [0 ball.State(13)*Ib(2,2)/Ib(1,1) -ball.State(12)*Ib(3,3)/Ib(1,1);...
                  -ball.State(13)*Ib(1,1)/Ib(2,2) 0 ball.State(11)*Ib(3,3)/Ib(2,2);...
                  ball.State(12)*Ib(1,1)/Ib(3,3) -ball.State(11)*Ib(2,2)/Ib(3,3) 0]*ball.State(11:13);
              
 commandMoment = -[Ib(1,1) 0 0;0 Ib(2,2) 0; 0 0 Ib(3,3)]*A2;
 thrust = A\commandMoment;
 
ball.Force = thrust(1)*[a1;b1;c1] + [thrust(2);0;0] + thrust(3)*[a2;b2;c2];
ball.Moment = commandMoment;
ball.State = ball.homebrewRK4();
if (mod(i,10) == 0)
    ball.State(4:7) = ball.State(4:7)*(1/norm(ball.State(4:7)));
end
testP(i) = ball.State(11);
testQ(i) = ball.State(12);
testR(i) = ball.State(13);
testF(i) = ball.Force(1)+ball.Force(2)+ball.Force(3);
testZ(i) = -ball.State(3);
 end
 
 