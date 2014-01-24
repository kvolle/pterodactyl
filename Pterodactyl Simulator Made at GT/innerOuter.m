
% Define the geometric angles in radians
phiL = 88*pi/180;
phiR = 92*pi/180;
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


a = 0.0013617439371;
b = 0.0808835077630;
c = 0.0336502034164;
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
 
A = [cross(position1,orientation1),cross(position2,orientation2),cross(position3,orientation3),cross(position4,orientation4)]+ 0.29*[orientation1,-orientation2,-orientation3,orientation4];
A(4,1) = orientation1(3);
A(4,2) = orientation2(3);
A(4,3) = orientation3(3);
A(4,4) = orientation4(3);
disp('Determinant');
det(A)

AA = [cross(position1,orientation1),cross(position2,orientation2),cross(position3,orientation3),cross(position4,orientation4)]+ 0.29*[orientation1,-orientation2,-orientation3,orientation4];
AA(4:6,:) = [orientation1 orientation2 orientation3 orientation4];
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
testRv=zeros(1,n);
testPv=zeros(1,n);
testYv=zeros(1,n);
testRa=zeros(1,n);
testPa=zeros(1,n);
testYa=zeros(1,n);

% Initialize 3 debugging variables
temp1 = zeros(1,n);
temp2 = zeros(1,n);
temp3 = zeros(1,n);
temp4 = zeros(1,n);

% PD Dynamics matrix
a = [0 1 0 0 0 0;-14 -10 0 0 0 0;0 0 0 1 0 0;0 0 -14 -10 0 0;0 0 0 0 0 1;0 0 0 0 -14 -10];
%a = [0 1 0 0 0 0;-14 -40 0 0 0 0;0 0 0 1 0 0;0 0 -1.2 -.8 0 0;0 0 0 0 0 1;0 0 0 0 -.924 -.66];
    testR(1) = copter.State(4);
    testP(1) = copter.State(5);
    testY(1) = copter.State(6);
    testRv(1) = copter.State(10);
    testPv(1) = copter.State(11);
    testYv(1) = copter.State(12);
    testRa(1) = 0;
    testPa(1) = 0;
    testYa(1) = 0;
   
    shit(1) = copter.State(3);
    
    time = 0.001:0.001:10;
    errorX = 0;
    errorY = 0;
    errorZ = 0;
    
    
for i=2:n 
    
    Or = copter.State(4:6);
    %{
    if (isnan(Or(1))||isnan(Or(2))||isnan(Or(3)))
        disp('Broken');
        i
        break
    end
    %}
    
    % Pre multiplying a vector in body coord by R transforms to world coord
    R = [cos(Or(2))*cos(Or(3)) -cos(Or(2))*sin(Or(3)) sin(Or(2));...
         cos(Or(1))*sin(Or(3))+cos(Or(3))*sin(Or(1))*sin(Or(2)) cos(Or(1))*cos(Or(3))-sin(Or(1))*sin(Or(2))*sin(Or(3)) -cos(Or(2))*sin(Or(1));...
         sin(Or(1))*sin(Or(3))-cos(Or(1))*cos(Or(3))*sin(Or(2)) cos(Or(3))*sin(Or(1))+cos(Or(1))*sin(Or(2))*sin(Or(3)) cos(Or(1))*cos(Or(2))];
     
   
    testR(i) = copter.State(4);
    testP(i) = copter.State(5);
    testY(i) = copter.State(6);
    
     % Now to calculate the total force required to maintain altitude
     tmp = R*copter.State(7:9);
     
     errorX = errorX + (copter.State(1)-0)/1000;
     errorY = errorY + (copter.State(2)-0)/1000;
     errorZ = errorZ + (copter.State(3)-setPoint)/1000;
     
     accelX = -15*(copter.State(1)-0) - 20*tmp(1) -10*errorX;
     accelY = -15*(copter.State(2)-0) - 20*tmp(2) -10*errorY;
     accelZ = -15*(copter.State(3)-setPoint) - 20*tmp(3) - 10*errorZ;
     
     %bodyForce = transpose(R)*m*[accelX;accelY;accelZ];
     worldForce = m*[accelX;accelY;accelZ];
     
     forceMag = norm(worldForce);
     forceDirection = [0;0;-1];
     if (forceMag > 0)
         forceDirection = worldForce/forceMag;
     end 
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    %Find the setpoints to align force with configuration force
    naturalForce = AA(4:6,:)*null(AA(1:3,:));
    naturalForce = naturalForce/norm(naturalForce);
    forceDirection;
    d = dot(naturalForce,forceDirection);
    theta = acos(d);

    w = cos(theta/2);

    c = cross(naturalForce,forceDirection);
    c = sin(theta/2)*c/norm(c);
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
    %{
    figure(4)
    hold on
    plot(i/1000,setRoll,'r.');
    plot(i/1000,setPitch,'g.');
    plot(i/1000,setYaw,'b.');
    %}
    if i == 2
        setRoll
        setPitch
        setYaw
    end
    % Calculate 
    x = [copter.State(4)-setRoll;copter.State(10);copter.State(5)-setPitch;copter.State(11);copter.State(6)-setYaw;copter.State(12)];
    f = a*x;
    
    
    % Calculate the moment to command in world coordinates
    worldMom = Ib*[f(2);f(4);f(6)];
    %%%Or = copter.State(4:6);

    % Pre multiplying a vector in body coord by R transforms to world coord
    %%%R = [cos(Or(2))*cos(Or(3)) -cos(Or(2))*sin(Or(3)) sin(Or(2));...
    %%%     cos(Or(1))*sin(Or(3))+cos(Or(3))*sin(Or(1))*sin(Or(2)) cos(Or(1))*cos(Or(3))-sin(Or(1))*sin(Or(2))*sin(Or(3)) -cos(Or(2))*sin(Or(1));...
    %%%     sin(Or(1))*sin(Or(3))-cos(Or(1))*cos(Or(3))*sin(Or(2)) cos(Or(3))*sin(Or(1))+cos(Or(1))*sin(Or(2))*sin(Or(3)) cos(Or(1))*cos(Or(2))];
     
     bodyMom = R\worldMom;
      

     thrust = A\[bodyMom;-forceMag];
     %thrust = A2\[bodyMom(1);bodyMom(2)];
     
     if max(abs(thrust))>15
         thrust = 15*thrust/max(abs(thrust));
     end
     temp1(i) = copter.State(1);%thrust(1);
     temp2(i) = copter.State(2);%thrust(2);
     temp3(i) = -copter.State(3);%thrust(3);
     
     shit(i) = copter.Force(1);%thrust(1);
     shit2(i) = copter.Force(2);%thrust(2);
     shit3(i) = copter.Force(3);%thrust(3);
     shit4(i) = thrust(4);

   %if mod(n,10)==0
        tmp = A*thrust;
        
        copter.Moment = tmp(1:3);%[thrust(1);0;thrust(2)];
        copter.Force = [orientation1 orientation2 orientation3 orientation4]*thrust;
    %end

     copter.State = copter.homebrewRK4();
end
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
%
plot3(temp1(2:3333),temp2(2:3333),temp3(2:3333),'r');
hold on
plot3(temp1(3334:6666),temp2(3334:6666),temp3(3334:6666),'g');
plot3(temp1(6667:10000),temp2(6667:10000),temp3(6667:10000),'b');
legend('First 3.3 Seconds','Second 3.3 Seconds','Final 3.3 Seconds');
%}
xlabel('X Displacement (ft)');
ylabel('Y Displacement (ft)');
zlabel('Altitude (ft)');
%}
figure(3)
plot(shit,'r');
hold on
plot(shit2,'g');
plot(shit3,'b');
%plot(shit4,'k');