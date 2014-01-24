clear all
clc

tic
n = 65;
nk = 5;
accelX = zeros(n,n,nk);
accelY = zeros(n,n,nk);
accelZ = zeros(n,n,nk);
scale = zeros(n,n,nk);
X = 1:n;
Y = 1:n;

hingeY = 2.4;
hinge2Prop = 2.4;

maximum = zeros(20,20);
maxX = zeros(20,20);
maxY = zeros(20,20);
maxZ = zeros(20,20);
bestPhiL = zeros(20,20);
bestPhiR = zeros(20,20);
bestPsi = zeros(20,20);
xx = zeros(1,20);
yy = zeros(1,20);
%{
for a = 1:20
    for b = 1:20
        hingeY = 0.5+0.1*a;
        hinge2Prop = 0.5+.2*b;
%}
%
for ii = 1:n
    for j = 1:n
        for k = 1:5
            phiL = (2*ii)*pi/180;
            X(ii) = 2*ii;
            phiR = (2*j)*pi/180;
            Y(j) = 2*j;
            psi = (3*k+15)*pi/180;
%}
        
            [comX,comY,comZ,Ix,Iy,Iz] = massCenter(15,phiL,phiR,psi,hingeY);
            
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
A(4:6,1) = orientation1;
A(4:6,2) = orientation2;
A(4:6,3) = orientation3;
A(4:6,4) = orientation4;

if (cond(A)<1000)
    scale(ii,j,k) = 1/cond(A);
else
    scale(ii,j,k) = NaN;
end
               
        end
    end
end


surf(X,Y,scale(:,:,5))
xlabel('phiL');
ylabel('phiR');

%{
[maximum(a,b),ind] = max(scale(:));
[tmp1,tmp2,tmp3] = ind2sub(size(scale),ind);
%maxX(a,b) = accelX(tmp1,tmp2,tmp3);
%maxY(a,b) = accelY(tmp1,tmp2,tmp3);
%maxZ(a,b) = accelZ(tmp1,tmp2,tmp3);
bestPhiL(a,b) = 80+0.5*tmp1;
bestPhiR(a,b) = 80+0.5*tmp2;
bestPsi(a,b) = 3*tmp3;

xx(a) = 0.5 + 0.1*a;
yy(b) = 0.5 + 0.1*b;
(a-1)*10+b
    end
end
surf(xx,yy,maximum)
%}
toc