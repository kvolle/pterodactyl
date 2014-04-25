clear all
clc

% Define the geometric angles in radians
phiL = 88*pi/180;
phiR = 92*pi/180;
psi = 15*pi/180;

% Distance from the center to the hinge along the leading edge
hingeY = 2.25;
% Distance from the hinge to propeller along the leading edge
hinge2Prop = 2;


% Defining the mass of the vehicle
m = 15/32.2; % Slugs, because FPS systems hate me

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

%}
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Find the direction of force when there is no torque applied
nullVector = null(A(1:3,:));
torqueFree = A(4:6,:)*nullVector;
torqueFree = torqueFree/norm(torqueFree);
% Calculate the roll, pitch and yaw angles for hover
d = dot(torqueFree,[0;0;-1]);
theta = acos(d);
c = cross(torqueFree,[0;0;-1]);
c = sin(theta/2)*c/norm(c);

w = cos(theta/2);
x = c(1);
y = c(2);
z = c(3);

w2 = w^2;
x2 = x^2;
y2 = y^2;
z2 = z^2;

Rd = [w2+x2-y2-z2 2*(x*y-w*z) 2*(w*y+x*z);2*(x*y+w*z) w2-x2+y2-z2 2*(y*z-w*x);2*(x*z-w*y) 2*(w*x+y*z) w2-x2-y2+z2];
setYaw = atan2(Rd(3,2),Rd(3,3))
setPitch = atan2(-Rd(3,1),Rd(3,3)/cos(setYaw))
setRoll = atan2(Rd(2,1),Rd(1,1))

%Instantiate the simulator
% For this code, we start with the vehicle in a random orientation
aa=random('unif',0,200)/100 - 1;
bb=random('unif',0,200)/100 - 1;
cc=random('unif',0,200)/100 - 1;
dd=random('unif',0,200)/100 - 1;

%{
setRoll = 0.002194;
setPitch = 0.076024;
setYaw = 0.000083;
%}
setPoint = -50;

%State = [-1;2;setPoint;aa;bb;cc;0;0;0;0;0;0];
State = [1;-3;setPoint;setRoll;setPitch;setYaw;0;0;0;0;0;0];
% Next line is for use if quaternions are used instead of euler angles
%State(4:7) = [cos(-pi/4);sin(-pi/4);0;0];
Force = [0;0;0];
Moment = [0;0;0];
copter = eulerRK4(State,Ib,m,Force,Moment);

% Number of miliseconds in simulation
n = 30000;

% Initialize angle, angle rate and angular accelration vectors
testR=zeros(1,n);
testP=zeros(1,n);
testY=zeros(1,n);

% Initialize 3 debugging variables
temp1 = zeros(1,n);
temp2 = zeros(1,n);
temp3 = zeros(1,n);
temp4 = zeros(1,n);

% PD Dynamics matrix
a = 2*[0 1 0 0 0 0;-14 -10 0 0 0 0;0 0 0 1 0 0;0 0 -14 -10 0 0;0 0 0 0 0 1;0 0 0 0 -14 -10];
%a = [0 1 0 0 0 0;-14 -40 0 0 0 0;0 0 0 1 0 0;0 0 -1.2 -.8 0 0;0 0 0 0 0 1;0 0 0 0 -.924 -.66];
    testR(1) = copter.State(4);
    testP(1) = copter.State(5);
    testY(1) = copter.State(6);

    errorX = 0;
    errorY = 0;
    errorZ = 0;
    
    time = 0.001:0.001:n/1000;

for i=2:n 
    temp1(i) = copter.State(1);
    temp2(i) = copter.State(2);
    temp3(i) = -copter.State(3);
    testR(i) = copter.State(4);
    testP(i) = copter.State(5);
    testY(i) = copter.State(6);

    Or = copter.State(4:6);
    if (isnan(Or(1))||isnan(Or(2))||isnan(Or(3)))
        break
    end
    % Pre multiplying a vector in body coord by R transforms to world coord
    R = [cos(Or(2))*cos(Or(3)) -cos(Or(2))*sin(Or(3)) sin(Or(2));...
         cos(Or(1))*sin(Or(3))+cos(Or(3))*sin(Or(1))*sin(Or(2)) cos(Or(1))*cos(Or(3))-sin(Or(1))*sin(Or(2))*sin(Or(3)) -cos(Or(2))*sin(Or(1));...
         sin(Or(1))*sin(Or(3))-cos(Or(1))*cos(Or(3))*sin(Or(2)) cos(Or(3))*sin(Or(1))+cos(Or(1))*sin(Or(2))*sin(Or(3)) cos(Or(1))*cos(Or(2))];


    % Now to calculate the total force required to maintain altitude
    tmp = R*copter.State(7:9);
    errorX = errorX + (copter.State(1)-0)/1000;
    errorY = errorY + (copter.State(2)-0)/1000;
    errorZ = errorZ + (copter.State(3)-setPoint)/1000;

    accel(1,1) = -0.4*(copter.State(1)) - .70*tmp(1) - .010*errorX;
    accel(2,1) = -4.6*(copter.State(2)) + 12.90*tmp(2) - .50*errorY;
    accel(3,1) = -1.5*(copter.State(3)-setPoint) - 3.0*tmp(3) - 0.05*errorZ;
accel(2) = -accel(2);
%    figure(3)
%    plot(time(i),copter.State(1),'r.');
%    hold on
%    plot(time(i),copter.State(2),'g.');
%    plot(time(i),copter.State(3)-setPoint,'b.');

    accel = accel - [0;0;32.2];


    worldForce = m*accel;
    copter.Force = transpose(R)*worldForce;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    forceMag = norm(worldForce);
    if (forceMag>0);
        forceDir = worldForce/forceMag;
    else
        forceDir = [0;0;-1];
    end

    %if (mod(i,20) == 1)
    d = dot(torqueFree,forceDir);
    theta = acos(d);
    c = cross(torqueFree,forceDir);
    c = sin(theta/2)*c/norm(c);

    w = cos(theta/2);
    x = c(1);
    y = c(2);
    z = c(3);

    w2 = w^2;
    x2 = x^2;
    y2 = y^2;
    z2 = z^2;

    Rd = [w2+x2-y2-z2 2*(x*y-w*z) 2*(w*y+x*z);2*(x*y+w*z) w2-x2+y2-z2 2*(y*z-w*x);2*(x*z-w*y) 2*(w*x+y*z) w2-x2-y2+z2];

    setYaw = atan2(Rd(3,2),Rd(3,3));
    setPitch = atan2(-Rd(3,1),Rd(3,3)/cos(setYaw));
    setRoll = atan2(Rd(2,1),Rd(1,1));
    
    if(abs(setYaw-copter.State(6))<0.00001)
        setYaw = copter.State(6);
    end
    if(abs(setPitch-copter.State(5))<0.00001)
        setPitch = copter.State(5);
    end
    if(abs(setRoll-copter.State(4))<0.00001)
        setRoll = copter.State(4);
    end
 
    %end
    x = [copter.State(4)-setRoll;copter.State(10);copter.State(5)-setPitch;copter.State(11);copter.State(6)-setYaw;copter.State(12)];
    f = a*x;
    
    % Calculate the moment to command in world coordinates
    worldMom = Ib*[f(2);f(4);f(6)];
    bodyMom = R\worldMom;

    for j=1:3
        if abs(bodyMom(j))<0.0001
            bodyMom(j) = 0;
        end
    end
    copter.Moment = bodyMom;

    thrust = [A(1:3,:);A(6,:)]\[bodyMom;forceMag*forceDir(3)];
 
     %thrust = A2\[bodyMom(1);bodyMom(2)];
     
     if max(abs(thrust))>15
         thrust = 15*thrust/max(abs(thrust));
     end
     

    %if mod(i,10)==2
        tmp = A*thrust;
        copter.Moment = tmp(1:3);%[thrust(1);0;thrust(2)];
        copter.Force = tmp(4:6);%[orientation1 orientation2 orientation3 orientation4]*thrust;
        wf = R*copter.Force;

        sue1(i) = wf(1)-forceMag*forceDir(1);
        sue2(i) = wf(2)-forceMag*forceDir(2);
        sue3(i) = wf(3)-forceMag*forceDir(3);
    %end
    %{
    if (mod(i,20) == 5)
    v1 = copter.State(1:3);
    v2 = v1+wf
    figure(3)
    plot3(v1(1),v1(2),-v1(3),'kh');
    hold on
    plot3([v1(1),v2(1)],[v1(2),v2(2)],[-v1(3),-v2(3)],'r');
    hold off
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal
    title(time(i))
    axis([-2 2 -2 2 49 66])
    end
%}
  
     
    

     copter.State = copter.homebrewRK4();
end
success = (norm(copter.State(4:6))<0.035);
err = norm(copter.State(4:6));
%
figure(1)
h1 = plot(time,testR*180/pi,'r',time,testP*180/pi,'g',time,testY*180/pi,'b');
legend('Roll Angle','Pitch Angle','Yaw Angle');
hold on
xlabel('Time (Seconds)');
ylabel('Angle (Degrees)');
%}
figure(2)
subplot(3,1,1)
plot(time,temp1,'r');
xlabel('Time(seconds)');
ylabel('Position(feet)');
axis([0 n/1000 -1 1])
title('X');
subplot(3,1,2)
plot(time,temp2,'g');
xlabel('Time(seconds)');
ylabel('Position(feet)');
axis([0 n/1000 -1 1])
title('Y');
subplot(3,1,3);
plot(time,temp3,'b');
xlabel('Time(seconds)');
ylabel('Position(feet)');
axis([0 n/1000 49 51])
title('Z')
%{
figure(3)
subplot(3,1,1)
plot(time,sue1,'r');
xlabel('Time(seconds)');
ylabel('Force');
axis([0 n/1000 -1 1])
title('X');
subplot(3,1,2)
plot(time,sue2,'g');
xlabel('Time(seconds)');
ylabel('Force');
axis([0 n/1000 -10 10])
title('Y');
subplot(3,1,3);
plot(time,sue3,'b');
xlabel('Time(seconds)');
ylabel('Force');
axis([0 n/1000 -1 1])
title('Z')
%}
%{
plot3(temp1(2:floor(n/3)),temp2(2:floor(n/3)),temp3(2:floor(n/3)),'r.');
hold on
plot3(temp1(floor(n/3)+1:2*floor(n/3)),temp2(floor(n/3)+1:2*floor(n/3)),temp3(floor(n/3)+1:2*floor(n/3)),'g.');
plot3(temp1(2*floor(n/3)+1:n),temp2(2*floor(n/3)+1:n),temp3(2*floor(n/3)+1:n),'b.');
legend('First 1/3','Second 1/3','Final 1/3');

xlabel('X Displacement (ft)');
ylabel('Y Displacement (ft)'); 
zlabel('Altitude (ft)');
axis([-1 1 -1 1 0 50])
axis equal
%}

%{
figure(3)
V1 = R*[1;0;0];
    V2 = R*[0;-1;0];
    V3 = R*[0;0;-1];
    plot3(linspace(0,V1(1),3),linspace(0,V1(2),3),linspace(0,V1(3),3),'r');
    axis([-1 1 -1 1 -1 1])
    hold on
    plot3(linspace(0,V2(1),3),linspace(0,V2(2),3),linspace(0,V2(3),3),'g');
    plot3(linspace(0,V3(1),3),linspace(0,V3(2),3),linspace(0,V3(3),3),'b');
copter.Force/norm(copter.Force)
%}