clear all
clc

m = 15;
phiL = 88*pi/180;
phiR = 92*pi/180;
psi = 15*pi/180;

hingeY = 2.25;
hinge2Prop = 2;

[comX,comY,comZ,Iz,Iy,Ix] = massCenter(m,phiL,phiR,psi,2.4);


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

 A = [cross(position1,orientation1),cross(position2,orientation2),cross(position3,orientation3),cross(position4,orientation4)] + 0.29*[orientation1,orientation2,-orientation3,-orientation4];
 A = [A;orientation1(3), orientation2(3), orientation3(3), orientation4(3)];
 det(A)
 
 A = [cross(position1,orientation1),cross(position2,orientation2),cross(position3,orientation3),cross(position4,orientation4)] + 0.29*[orientation1,-orientation2,-orientation3,orientation4];
 A = [A;orientation1(3), orientation2(3), orientation3(3), orientation4(3)];
 det(A)
 
 A = [cross(position1,orientation1),cross(position2,orientation2),cross(position3,orientation3),cross(position4,orientation4)] + 0.29*[-orientation1,orientation2,orientation3,-orientation4];
 A = [A;orientation1(3), orientation2(3), orientation3(3), orientation4(3)];
 det(A)
 
 A = [cross(position1,orientation1),cross(position2,orientation2),cross(position3,orientation3),cross(position4,orientation4)] + 0.29*[-orientation1,-orientation2,orientation3,orientation4];
 A = [A;orientation1(3), orientation2(3), orientation3(3), orientation4(3)];
 det(A)
%}