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
            psi = (3*k)*pi/180;
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

            A = [cross(position1,orientation1),cross(position2,orientation2),cross(position3,orientation3)]+0.13*[orientation1,[0;0;-1],-orientation3];
            %if cond(A) <10000
                scale(ii,j,k) = det(A);
                accelZ(ii,j,k) = abs(A(3,1))+abs(A(3,2))+abs(A(3,3));
                %test(ii,j,k) = norm(A(:,2));
                %accelY(ii,j,k) = norm(A(:,2));
                if abs(scale(ii,j,k))<0.01
                    scale(ii,j,k) = NaN;
                    accelZ(ii,j,k) = NaN;
                    %test(ii,j,k) = NaN;
                end
                %{
                A = abs(A);
                accelX(i,j,k) = sum(A(1,:))/Ix;
                accelY(i,j,k) = sum(A(2,:))/Iy;
                accelZ(i,j,k) = sum(A(3,:))/Iz;
                scale(i,j,k) = norm([accelX(i,j,k) accelY(i,j,k) accelZ(i,j,k)]);
                X(i) = phiL;
                Y(j) = phiR;
                %}
            %else
               % accelX(i,j,k) = NaN;
               % accelY(i,j,k) = NaN;
               % accelZ(i,j,k) = NaN;
               % scale(i,j,k) = NaN;
            %end            
        end
    end
end

surf(X,Y,accelZ(:,:,5))
xlabel('phiL');
ylabel('phiR');
figure(2)
surf(X,Y,scale(:,:,5))
xlabel('phiL');
ylabel('phiR');
%figure(3)
%surf(X,Y,test(:,:,5))
%xlabel('phiL');
%ylabel('phiR');

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