clear all
clc
m = 15;

Mx = zeros(13,14,5);
My = zeros(13,14,5);
Mz = zeros(13,14,5);
condTable = zeros(13,14,5);

hingeY = 1.75;
hinge2Prop = 0.75;

for i = 1:13
    %x(i) = NaN;
    for j = 1:14
        %phiL(i) = NaN;
        for k = 1:5
            %psi(k) = NaN;
            
            condTable(i,j,k) = NaN;
            Mx(i,j,k) = NaN;
            My(i,j,k) = NaN;
            Mz(i,j,k) = NaN;
        end
    end
end


for i = 1:13
    phiL = 5*i +55;
    for j = i:14
        phiR = 5*i+60;
        tic
        for k =1:5
            psi = k*3 -2;
            
            [comX,comY,comZ,Ix,Iy,Iz] = massCenter(m,phiL,phiR,psi);
            
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

A = [cross(position1,orientation1),cross(position2,[0;0;-1]),cross(position3,orientation3)];
            Mx(i,j,k) = (abs(A(1,1))+abs(A(1,2))+abs(A(1,3)))/Ix;
            My(i,j,k) = (abs(A(2,1))+abs(A(2,2))+abs(A(2,3)))/Iy;
            Mz(i,j,k) = (abs(A(3,1))+abs(A(3,2))+abs(A(3,3)))/Iz;
            condTable(i,j,k) = 1/cond(A);

        end
        toc
    end
end