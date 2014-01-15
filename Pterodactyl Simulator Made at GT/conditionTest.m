clear all
clc

hingeY = 1.75;
hinge2Prop = 1;
m = 15;
invert = 2*ones(16,16,13);
for i = 1:16
    for j = 1:16
        for k = 1:4
            phiL = 50+5*i;
            phiR = 50+5*j;
            psi  = 4*k-3;
 
% Left wing thrust coefficients
[comX,comY,comZ,Ix,Iy,Iz] = massCenter(m,phiL,phiR,psi);

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
position2 = [comZ;-comY;-comX];% DOUBLE CHECK

% A matrix converts thrusts to moments        

 A = [(-position1(3)*b1+position1(2)*c1) -position2(2) (-position3(3)*b2+position3(2)*c2);...
     (position1(3)*a1-position1(1)*c1) -position2(1) position3(3)*a2-position3(1)*c2;...
     (position1(1)*b1-a1*position1(2)) 0 (-a2*position3(2)+position3(1)*b2)];
 
 
 invert(i,j,k) = 1/cond(A);
        end
    end
end
