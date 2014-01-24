clear all
clc

% Define the geometric angles in radians
phiL = 130*pi/180;
phiR = 130*pi/180;
psi = 15*pi/180;

% Distance from the center to the hinge along the leading edge
hingeY = 2.25;
% Distance from the hinge to propeller along the leading edge
hinge2Prop = 2;

% For this code, we start with the vehicle in a random orientation
a=random('unif',0,200)/100 - 1;
b=random('unif',0,200)/100 - 1;
c=random('unif',0,200)/100 - 1;
d=random('unif',0,200)/100 - 1;


a = 0;
b = 0;
c = 0;
setPoint = -50;

State = [0;0;setPoint;a;b;c/2;0;0;0;0;0;0];
% Next line is for use if quaternions are used instead of euler angles
%State(4:7) = [cos(-pi/4);sin(-pi/4);0;0];

% Defining the mass of the vehicle and 
m = 15/32.2; % Slugs, because FPS systems hate me
Force = [0;0;0];
Moment = [0;0;0];

% Mass center code approximates center of mass and moment of inertia,
% assuming uniform vehicle density and ignoring triangular sections
% TODO fix mass center code
[comX,comY,comZ,Iz,Iy,Ix] = massCenter(m*32.2,phiL,phiR,psi,hingeY);
Ix = 30.93;
Iy = 22.43;
Iz = 30.68;
Ib = [Ix 0 0;0 Iy 0;0 0 Iz];


%~~~~~~~~~~~~~~~~~~~~~~~~~Counter Rotating~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%{
 tmp =[0 0 1 -comZ;0 1 0 -comY;-1 0 0 comX;0 0 0 1]*...
      [1 0 0 0;0 1 0 -hingeY;0 0 1 0;0 0 0 1]*...
      [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0;0 0 1 0;0 0 0 1]*...
      [1 0 0 0;0 cos(-phiL) -sin(-phiL) 0;0 sin(-phiL) cos(-phiL) 0;0 0 0 1]*...
      [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0;0 0 1 0;0 0 0 1]*...
      [1 0 0 0;0 1 0 -hinge2Prop;0 0 1 0;0 0 0 1];
 orientation1 = tmp(1:3,1);
 position1 = tmp(1:3,4);
 orientation2 = [0;0;-1];
 position2 = [-comZ;-comY;comX];
 tmp =[0 0 1 -comZ;0 1 0 -comY;-1 0 0 comX;0 0 0 1]*...
      [1 0 0 0;0 1 0 hingeY;0 0 1 0;0 0 0 1]*...
      [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0;0 0 1 0;0 0 0 1]*...
      [1 0 0 0;0 cos(phiR) -sin(phiR) 0;0 sin(phiR) cos(phiR) 0;0 0 0 1]*...
      [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0;0 0 1 0;0 0 0 1]*...
      [1 0 0 0;0 1 0 hinge2Prop;0 0 1 0;0 0 0 1];
 orientation3 = tmp(1:3,1);
 position3 = tmp(1:3,4);
 orientation4 = [0;0;-1];
 position4 = [-comZ;-comY;comX];

 A = [cross(position1,orientation1),cross(position2,orientation2),cross(position3,orientation3),cross(position4,orientation4)] + 0.29*[orientation1,-orientation2,-orientation3,orientation4];
 A = [A;orientation1(3), orientation2(3), orientation3(3), orientation4(3)]
 det(A)
%}
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%~~~~~~~~~~~~~~~~~~~~~~~~~Modified Design~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%
 tmp =[0 0 1 -comZ;0 1 0 -comY;-1 0 0 comX;0 0 0 1]*...
      [1 0 0 0;0 1 0 -hingeY;0 0 1 0;0 0 0 1]*...
      [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0;0 0 1 0;0 0 0 1]*...
      [1 0 0 0;0 cos(-phiL) -sin(-phiL) 0;0 sin(-phiL) cos(-phiL) 0;0 0 0 1]*...
      [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0;0 0 1 0;0 0 0 1]*...
      [1 0 0 0;0 1 0 -hinge2Prop;0 0 1 0;0 0 0 1];
 orientation1 = tmp(1:3,1);
 position1 = tmp(1:3,4);

 tmp = [0 0 1 -comZ;0 1 0 -comY;-1 0 0 comX;0 0 0 1]*...
       [1 0 0 0;0 1 0 -2;0 0 1 0;0 0 0 1];
 orientation2 = tmp(1:3,1);
 position2 = tmp(1:3,4);
 
 tmp = [0 0 1 -comZ;0 1 0 -comY;-1 0 0 comX;0 0 0 1]*...
       [1 0 0 0;0 1 0 2;0 0 1 0;0 0 0 1];
 orientation3 = tmp(1:3,1);
 position3 = tmp(1:3,4);
 
 tmp =[0 0 1 -comZ;0 1 0 -comY;-1 0 0 comX;0 0 0 1]*...
      [1 0 0 0;0 1 0 hingeY;0 0 1 0;0 0 0 1]*...
      [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0;0 0 1 0;0 0 0 1]*...
      [1 0 0 0;0 cos(phiR) -sin(phiR) 0;0 sin(phiR) cos(phiR) 0;0 0 0 1]*...
      [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0;0 0 1 0;0 0 0 1]*...
      [1 0 0 0;0 1 0 hinge2Prop;0 0 1 0;0 0 0 1];
 orientation4 = tmp(1:3,1);
 position4 = tmp(1:3,4);
 
A = [cross(position1,orientation1),cross(position2,orientation2),cross(position3,orientation3),cross(position4,orientation4)]+ 0.29*[orientation1,-orientation2,orientation3,-orientation4];
A(4:6,1) = orientation1;
A(4:6,2) = orientation2;
A(4:6,3) = orientation3;
A(4:6,4) = orientation4;
AA = A(1:3,:);
AA(4,:) = A(6,:);
AAA = [A(1:2,:);A(4:6,:)];
%}
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

%Instantiate the simulator
copter = eulerRK4(State,Ib,m,Force,Moment);

% Number of miliseconds in simulation
n = 10000;

% Initialize angle, angle rate and angular accelration vectors

testR=zeros(1,n);
testP=zeros(1,n);
testY=zeros(1,n);
%{
testRv=zeros(1,n);
testPv=zeros(1,n);
testYv=zeros(1,n);
testRa=zeros(1,n);
testPa=zeros(1,n);
testYa=zeros(1,n);
%}

% Initialize 3 debugging variables
temp1 = zeros(1,n);
temp2 = zeros(1,n);
temp3 = zeros(1,n);
temp4 = zeros(1,n);
temp3 = -setPoint;
mag = zeros(1,n);
mag(1) = NaN;

time = 0.001:0.001:n/1000;

% PD Dynamics matrix
a = [0 1 0 0 0 0;-14 -10 0 0 0 0;0 0 0 1 0 0;0 0 -14 -10 0 0;0 0 0 0 0 1;0 0 0 0 -14 -10];
%a = [0 1 0 0 0 0;-14 -40 0 0 0 0;0 0 0 1 0 0;0 0 -1.2 -.8 0 0;0 0 0 0 0 1;0 0 0 0 -.924 -.66];

    testR(1) = copter.State(4);
    testP(1) = copter.State(5);
    testY(1) = copter.State(6);
  %{
    testRv(1) = copter.State(10);
    testPv(1) = copter.State(11);
    testYv(1) = copter.State(12);
    testRa(1) = 0;
    testPa(1) = 0;
    testYa(1) = 0;
  %}  
    time(1) = 0.001;
    
    errorX = 0;
    errorY = 0;
    errorZ = 0;
    
    shit(1) = copter.State(3);
for i=2:n 

   
    testR(i) = copter.State(4);
    testP(i) = copter.State(5);
    testY(i) = copter.State(6);
    %{
    testRv(i) = copter.State(10);
    testPv(i) = copter.State(11);
    testYv(i) = copter.State(12);
    testRa(i) = 1000*(testRv(i)-testRv(i-1));
    testPa(i) = 1000*(testPv(i)-testPv(i-1));
    testYa(i) = 1000*(testYv(i)-testYv(i-1));
    %}
    x = [copter.State(4);copter.State(10);copter.State(5);copter.State(11);copter.State(6);copter.State(12)];
    f = a*x;
    
    % Calculate the moment to command in world coordinates
    worldMom = Ib*[f(2);f(4);f(6)];
    Or = copter.State(4:6);
    % Pre multiplying a vector in body coord by R transforms to world coord
    R = [cos(Or(2))*cos(Or(3)) -cos(Or(2))*sin(Or(3)) sin(Or(2));...
         cos(Or(1))*sin(Or(3))+cos(Or(3))*sin(Or(1))*sin(Or(2)) cos(Or(1))*cos(Or(3))-sin(Or(1))*sin(Or(2))*sin(Or(3)) -cos(Or(2))*sin(Or(1));...
         sin(Or(1))*sin(Or(3))-cos(Or(1))*cos(Or(3))*sin(Or(2)) cos(Or(3))*sin(Or(1))+cos(Or(1))*sin(Or(2))*sin(Or(3)) cos(Or(1))*cos(Or(2))];
     bodyMom = R\worldMom;

     if (isnan(Or(1))||isnan(Or(2))||isnan(Or(3)))
         break
     end
     % Now to calculate the total force required to maintain altitude
     tmp = R*copter.State(7:9);
     errorX = errorX + (copter.State(1))/1000;
     errorY = errorY + (copter.State(2))/1000;
     errorZ = errorZ + (copter.State(3)-setPoint)/1000;
     accelX = -15*(copter.State(1)) - 20*tmp(1) -10*errorX;
     accelY = -15*(copter.State(2)) - 20*tmp(2) -10*errorY;
     accelZ = -15*(copter.State(3)-setPoint) - 20*tmp(3) - 10*errorZ;
     
     bodyForce = transpose(R)*m*[accelX;accelY;accelZ-15];

     thrust = pinv(A)*[bodyMom;bodyForce];
     %thrust = pinv(AAA)*[bodyMom(1:2,1);bodyForce];
     %thrust = AA\[bodyForce;bodyMom(3)];
     %thrust = AA\[bodyMom;bodyForce(3)];
     
     t = A*thrust-[bodyMom;bodyForce];
     mag(i) = sqrt(t(1)^2 + t(2)^2 + t(3)^2 + t(4)^2 + t(5)^2 + t(6)^2);
     
     if (i == 200)
       thrust
       AA*thrust
     end
     
     if max(abs(thrust))>15
         thrust = 15*thrust/max(abs(thrust));
     end
     temp1(i) = copter.State(1);%thrust(1);
     temp2(i) = copter.State(2);%thrust(2);
     temp3(i) = -copter.State(3);%thrust(3);
     shit(i) = thrust(1);
     shit2(i) = thrust(2);
     shit3(i) = thrust(3);
     shit4(i) = thrust(4);

     
    if mod(i,10)==0
        tmp = A*thrust;
        copter.Moment = tmp(1:3);
        copter.Force = tmp(4:6);%[orientation1 orientation2 orientation3 orientation4]*thrust;
    end

     copter.State = copter.homebrewRK4();
end

i
success = (norm(copter.State(4:6))<0.035);
err = norm(copter.State(4:6));
%
figure(1)
plot(time,testR*180/pi,'r',time,testP*180/pi,'g',time,testY*180/pi,'b');
legend('Roll Angle','Pitch Angle','Yaw Angle');
xlabel('Time (Seconds)');
ylabel('Angle (Degrees)');
%}

figure(2)

plot(time,temp3,'k');
axis([0 10 0 55])

%{
plot3(temp1(2:3333),temp2(2:3333),temp3(2:3333),'r');
hold on
plot3(temp1(3334:6666),temp2(3334:6666),temp3(3334:6666),'g');
plot3(temp1(6667:10000),temp2(6667:10000),temp3(6667:10000),'b');
legend('First 3.3 Seconds','Second 3.3 Seconds','Final 3.3 Seconds');

xlabel('X Displacement (ft)');
ylabel('Y Displacement (ft)');
zlabel('Altitude (ft)');
%}
%{
figure(3)
plot(shit+2,'r');
hold on
plot(shit2,'g');
plot(shit3,'b');
plot(shit4,'k');
%}

%figure(3)
%plot(time,mag)
