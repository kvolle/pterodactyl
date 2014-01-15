function [stability,err] = test(phiL,phiR,psi,tl)


psi = psi*pi/180;
phiL = phiL*pi/180;
phiR = phiR*pi/180;
%}
k = 30;
d = 60;

m = 15;

[comX,comY,comZ,Iz,Iy,Ix] = massCenter(m,phiL,phiR,psi);
Ix%= abs(Ix)
Iy%= abs(Iy)
Iz%= abs(Iz)
%{
Ix = 63;
Iy =6.3;
Iz = 63;
%}
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
orientation3 = tmp(1:3,1);
position3 = tmp(1:3,4);
position2 = [-comZ;-comY;-comX];% DOUBLE CHECK

% A matrix converts thrusts to moments        
%{
I believe this one to be incorrect with regards to thruster 2
A = [(-position1(3)*b1+position1(2)*c1) 0 (-position3(3)*b2+position3(2)*c2);...
     (position1(3)*a1-position1(1)*c1) position2(3) position3(3)*a2-position3(1)*c2;...
     (position1(1)*b1-a1*position1(2)) -position2(2) (-a2*position3(2)+position3(1)*b2)]
 %
 A = [(-position1(3)*b1+position1(2)*c1) -position2(2) (-position3(3)*b2+position3(2)*c2);...
     (position1(3)*a1-position1(1)*c1) -position2(1) position3(3)*a2-position3(1)*c2;...
     (position1(1)*b1-a1*position1(2)) 0 (-a2*position3(2)+position3(1)*b2)]
%}

A = [cross(position1,orientation1),cross(position2,[0;0;-1]),cross(position3,orientation3)]
abs(A)
 % Simulator setup
 a = random('unif',0,200)/100 -1;
 b = random('unif',0,200)/100 -1;
 c = random('unif',0,200)/100 -1;
 
State = [0;0;-100;a/2;b/2;c/2;0;0;0;-a;-b;-c];
%State = [0;0;-100;0;0;0.0;0;0;0;0;0;0];

Ib = [Ix 0 0;0 Iy 0;0 0 Iz];

Force = [0;0;0];
Moment = [0;0;0];

size= 5000;

testR = zeros(1,size);
testP = zeros(1,size);
testY = zeros(1,size);
testZ = zeros(1,size);
shit1 = zeros(1,size);
shit2 = zeros(1,size);
shit3 = zeros(1,size);
shit4 = zeros(1,size);
shit = zeros(1,size);
t = zeros(1,size);


copter = eulerRK4(State,Ib,m,Force,Moment);
%a = [0 1 0 0 0 0;-.60 -.40 0 0 0 0;0 0 0 1 0 0;0 0 -.60 -.40 0 0; 0 0 0 0 0 1;0 0 0 0 -.60 -.40]
cs = 8.3;
%a = [0 1 0 0 0 0;-.8 -.6 0 0 0 0;0 0 0 1 0 0;0 0 -.8 -.6 0 0;0 0 0 0 0 1;0 0 0 0 -.8 -.6];
%a = [0 1 0 0 0 0;-2/Ix -1.5/Ix 0 0 0 0;0 0 0 1 0 0;0 0 2/Iy -1.5/Iy 0 0;0 0 0 0 0 1;0 0 0 0 -2/Iz -1.5/Iz];
a = [0 1 0 0 0 0;-5 -3 0 0 0 0;0 0 0 1 0 0;0 0 -5 -3 0 0;0 0 0 0 0 1;0 0 0 0 -5 -10];


 for i =1:size
     t(i) = i/1000;
     testR(i) = copter.State(4);
     testP(i) = copter.State(5);
     testY(i) = copter.State(6);
     testZ(i) = -copter.State(3);
     if mod(i, 10) == 1
         
        x = [copter.State(4);copter.State(10);copter.State(5);copter.State(11);copter.State(6);copter.State(12)];
        Or = copter.State(4:6);
        R = [cos(Or(2))*cos(Or(3)) -cos(Or(2))*sin(Or(3)) sin(Or(2));...
        cos(Or(1))*sin(Or(3))+cos(Or(3))*sin(Or(1))*sin(Or(2)) cos(Or(1))*cos(Or(3))-sin(Or(1))*sin(Or(2))*sin(Or(3)) -cos(Or(2))*sin(Or(1));...
        sin(Or(1))*sin(Or(3))-cos(Or(1))*cos(Or(3))*sin(Or(2)) cos(Or(3))*sin(Or(1))+cos(Or(1))*sin(Or(2))*sin(Or(3)) cos(Or(1))*cos(Or(2))];
        axis = R\[1;0;0];
        iwx = dot(axis, [Ix 0 0;0 Iy 0;0 0 Iz]*axis);
        axis = R\[0;1;0];
        iwy = dot(axis, [Ix 0 0;0 Iy 0;0 0 Iz]*axis);
        axis = R\[0;0;1];
        iwz = dot(axis, [Ix 0 0;0 Iy 0;0 0 Iz]*axis);

        %{
        body2World = [cos(Or(1))*cos(Or(2)) cos(Or(1))*sin(Or(2))*sin(Or(3))-sin(Or(1))*cos(Or(3)) cos(Or(1))*sin(Or(2))*cos(Or(3))+sin(Or(1))*sin(Or(3));...
                       sin(Or(1))*cos(Or(2)) sin(Or(1))*sin(Or(2))*sin(Or(3))+cos(Or(1))*cos(Or(3)) sin(Or(1))*sin(Or(2))*cos(Or(3))-cos(Or(1))*sin(Or(3));...
                       -sin(Or(2)) cos(Or(2))*sin(Or(3)) cos(Or(2))*cos(Or(3))];
        %}
        f = a*x;
       % b = [0 1;-7 -7.2]*[copter.State(3);copter.State(9)];
        %worldForce = [0;0;b(2)*m];
     %
       % bodyForce = body2World\worldForce;
     
         
        Mx = f(2)*iwx/1;
        My = f(4)*iwy/1;
        Mz = f(6)*iwz/1;
        %
        g = [f(2);f(4);f(6)];
        %{
        Mom = body2World\g;
        Mom = [Ix 0 0;0 Iy 0;0 0 Iz]*Mom;
        Mx = Mom(1);
        My = Mom(2);
        Mz = Mom(3);
        %}
        shit1(i:i+9) = g(1);
        shit2(i:i+9) = g(2);
        shit3(i:i+9) = g(3);

        bodyMom = R\[Mx;My;Mz];
        maxMom = abs(A)*[tl-0.5;2*(tl-0.5);tl-0.5];
        if bodyMom(1) > maxMom(1)/2
          bodyMom(1) = maxMom(1)/2;
        elseif bodyMom(1) <-maxMom(1)/2
          bodyMom(1) = -maxMom(1)/2;
        end
        if bodyMom(2) > maxMom(2)/2
          bodyMom(2) = maxMom(2)/2;
        elseif bodyMom(2) <-maxMom(2)/2
          bodyMom(2) = -maxMom(2)/2;
        end
        if bodyMom(3) >maxMom(3)/2
          bodyMom(3) = maxMom(3)/2;
        elseif bodyMom(3) <-maxMom(3)/2
          bodyMom(3) = -maxMom(3)/2;
        end
transpose(bodyMom)
        %thrust = [0;0;0;0];
        thrust = A\bodyMom;
%{        
        A = [A,A(:,2)];
        A = A+[0.06368*orientation1, 0.06368*[0;0;1],-0.06368*orientation2,0.06368*[0;0;-1]];
    	A = [A;[
        if (Mx >0.01)||(My>0.01)||(Mz>0.01)
          thrust = A\[Mx;My;Mz;bodyForce(3)];
        end
%}
     %
        disp('Desired');
        transpose(thrust)
        
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
     %}
     disp('Given');
     transpose(thrust)
     
          copter.Moment = A*thrust;%+torqueCalculator(thrust(1))*orientation1 + 0*[0;0;-1] + -torqueCalculator(thrust(3))*orientation3;
          copter.Force = thrust(1)*[a1;b1;c1] + [0; 0; -thrust(2)] + thrust(3)*[a2;b2;c2]; %MIDDLE TERM CORRECT?
          %tmp = body2World*copter.Force;
     end

     copter.State = copter.homebrewRK4();
     

 end
 err = norm(copter.State(4:6));
stability = (err<0.005);

%
 plot(testR*180/pi,'r');
 hold on
 plot(testP*180/pi,'g');
 plot(testY*180/pi,'b');
 xlabel('Time (ms)');
 ylabel('Displacement (Degrees)');
%} 
%{
figure(2)
plot(shit1,'r.');
hold on
plot(shit2,'g.');
plot(shit3,'b.');
%}
end