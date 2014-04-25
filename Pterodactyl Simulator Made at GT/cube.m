clear all
clc
col = ['r','g','b','c','y','k'];
x = 12*[-1 1 1 -1;-1 -1 -1 -1;-1 1 1 -1;1 1 1 1;1 -1 -1 1;-1 -1 1 1];
y = 12*[-1 -1 -1 -1;-1 -1 1 1;-1 -1 1 1;-1 -1 1 1;-1 -1 1 1;1 1 1 1];
z = 12*[-1 -1 1 1;-1 1 1 -1;-1 -1 -1 -1;-1 1 1 -1;1 1 1 1;-1 1 1 -1];

%x = [0 1 1 0;0 0 0 0;0 1 1 0;1 1 1 1;1 0 0 1;0 0 1 1];
%y = [0 0 0 0;0 0 1 1;0 0 1 1;0 0 1 1;0 0 1 1;1 1 1 1];
%z = [0 0 1 1;0 1 1 0;0 0 0 0;0 1 1 0;1 1 1 1;0 1 1 0];
size(x)

A =[ 1.885543506849142  -0.022069236893791  -1.590338480554133;
   2.977929901099993                   0   2.983048602652628;
   1.271615903764178  -0.013000000000000  -1.285247992399106];

det(A)


%A = [1 0 0;0 1 0;0 0 1];

for i =1:6
    for j = 1:4
        tmp = A*[x(i,j);y(i,j);z(i,j)];
        x1(i,j) = tmp(1);
        y1(i,j) = tmp(2);
        z1(i,j) = tmp(3);
    end
end

figure(1)
for i = 1:6
    h = patch(x(i,:),y(i,:),z(i,:),'w');
    set(h,'facecolor',col(i));
end
view(3)
xlabel('Thruster 1')
ylabel('Thruster 2')
zlabel('Thruster 3')

figure(2)

for i = 1:6
    h = patch(x1(i,:),y1(i,:),z1(i,:),'w');
    set(h,'facecolor',col(i));
end
view(3)
xlabel('Roll')
ylabel('Pitch')
zlabel('Yaw')
%{
hold on
xprime = A*[2;0;0];
plot3(linspace(0,xprime(1)),linspace(0,xprime(2)),linspace(0,xprime(3)),'r');
yprime = A*[0;2;0];
plot3(linspace(0,yprime(1)),linspace(0,yprime(2)),linspace(0,yprime(3)),'g');
zprime = A*[0;0;2];
plot3(linspace(0,zprime(1)),linspace(0,zprime(2)),linspace(0,zprime(3)),'b');
%}