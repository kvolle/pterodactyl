function [stability,err] = test(phiL,phiR,psi,tl)

psi = psi*pi/180;
phiL = phiL*pi/180;
phiR = phiR*pi/180;

m = 15;

[comX,comY,comZ,Ix,Iy,Iz] = massCenter(m,phiL,phiR,psi);

hingeY = 1.75;
hinge2Prop = 1;

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
orientation3 = tmp(1:3,1)
position3 = tmp(1:3,4)
position2 = [-comZ;-comY;-comX];% DOUBLE CHECK

% A matrix converts thrusts to moments        
%{
 A = [(-position1(3)*b1+position1(2)*c1) -position2(2) (-position3(3)*b2+position3(2)*c2);...
     (position1(3)*a1-position1(1)*c1) -position2(1) position3(3)*a2-position3(1)*c2;...
     (position1(1)*b1-a1*position1(2)) 0 (-a2*position3(2)+position3(1)*b2)]
%}
 A = [cross(position1,orientation1),cross(position2,[0;0;-1]),cross(position3,orientation3)];

 % Simulator setup
 a = random('unif',0,100)/100;
 b = random('unif',0,200)/100 -1;
 c = random('unif',0,200)/100 -1;
 d = random('unif',0,200)/100 -1;
 
State = [0;0;-100;cos(a/2);sin(a/2)*b/norm([b;c;d]);sin(a/2)*c/norm([b;c;d]);sin(a/2)*d/norm([b;c;d]);0;0;0;-a;-b;-c];
Ix = 1;
Iy = 1;
Iz = 1;
Ib = [Ix 0 0;0 Iy 0;0 0 Iz];

Force = [0;0;0];
Moment = [0;0;0];

size= 5000;

testS = zeros(1,size);
testP = zeros(1,size);
testC = zeros(1,size);
testZ = zeros(1,size);
shit1 = zeros(1,size);
shit2 = zeros(1,size);
shit3 = zeros(1,size);
shit4 = zeros(1,size);
shit = zeros(1,size);
mid = ones(1,size)*180;
t = zeros(1,size);

copter = quatRK4(State,Ib,m,Force,Moment);


%qOld = 2*acos(copter.State(4));
qOld = 2*atan2(norm(copter.State(5:7)),copter.State(4)); 
for i =1:size
     t(i) = i/1000;
     testP(i) = 2*atan2(norm(copter.State(5:7)),copter.State(4));;
     testS(i) = norm(copter.State(5:7));
     testC(i) = copter.State(4);

     testZ(i) = copter.State(4);

     if mod(i, 10) <11
        axis = copter.State(5:7)/norm(copter.State(5:7));
        inertia = dot(axis, [Ix 0 0;0 Iy 0;0 0 Iz]*axis)
        a = [0 1;-20 -.3];

        x = [testP(i);1000*(testP(i)-qOld)];
                
        qOld = testP(i);
        f = a*x;
        shit(i:i+9) = f(1)/testP(i);


        Mom = f(2)*inertia*axis;
        
        q = copter.State(4:7);

        R = [1-2*(q(3)^2+q(4)^2) 2*(q(2)*q(3)-q(1)*q(4)) 2*(q(2)*q(4)+q(1)*q(3));...
             2*(q(2)*q(3)+q(1)*q(4)) 1-2*(q(2)^2+q(4)^2) 2*(q(3)*q(4)-q(1)*q(2));...
             2*(q(2)*q(4)-q(1)*q(3)) 2*(q(3)*q(4)+q(1)*q(2)) 1-2*(q(2)^2+q(3)^2)];
        shit3(i) = atan2(R(3,2),R(3,3));
        shit2(i) = atan2(-R(3,1),R(3,3)/cos(shit3(i)));
        shit1(i) = atan2(R(2,1),R(1,1));
             Mom = R\Mom;

             thrust = A\Mom;
             
        if thrust(1) >tl
          thrust(1) = tl;
        elseif thrust(1) <-tl
          thrust(1) = -tl;
        end
        if thrust(2) >2*tl
          thrust(2) = 2*tl;
        elseif thrust(2) <-2*tl
          thrust(2) = -2*tl;
        end
        if thrust(3) >tl
          thrust(3) = tl;
        elseif thrust(3) <-tl
          thrust(3) = -tl;
        end

          copter.Moment = A*thrust;%+torqueCalculator(thrust(1))*orientation1 + 0*[0;0;-1] + -torqueCalculator(thrust(3))*orientation3;
          copter.Force = thrust(1)*[a1;b1;c1] + [0; 0; -thrust(2)] + thrust(3)*[a2;b2;c2]; %MIDDLE TERM CORRECT?
     end

     copter.State = copter.homebrewRK4();
     %copter.State(4:7) = copter.State(4:7)/sqrt(transpose(copter.State(4:7))*eye(4)*copter.State(4:7));
     if (abs(1-transpose(copter.State(4:7))*eye(4)*copter.State(4:7)>0.00000001))
         copter.State(5:7) = sqrt(1-copter.State(4))*copter.State(5:7)/norm(copter.State(5:7));
     end

 end
 err = norm(copter.State(4:6));
stability = (err<0.005);

 hold on

 plot(shit3*180/pi,'r');
 plot(shit2*180/pi,'g');
 plot(shit1*180/pi,'b');
 plot(shit4,'k');
 xlabel('Time (ms)');
 ylabel('Displacement (Degrees)');
 figure(2)
 plot(testP*180/pi,'g');

end