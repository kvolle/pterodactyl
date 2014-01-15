clear all
clc

hold on
state = [-2;0;0;0;0;0;0;0;0;0;0;0];
Ib = [1 0 0;0 1 0;0 0 1];
m = 15;
Force = [0;0;0];
Moment = [0;0;0];

ball = eulerRK4(state,Ib,m,Force,Moment);

size = 5000;
force = zeros(1,size);
shit = zeros(1,size);
t = zeros(1,size);
x0 = [ball.State(1);ball.State(7)+5];


A = [0 1;-2 -0];


for i = 1:size;
t(i) = i/1000;
x = [ball.State(1);ball.State(7)];
Rg = inv(A)*expm(A*2)*[0 0;0 1]*inv(transpose(A))*expm(transpose(A)*2)-inv(A)*[0 0;0 1]*inv(transpose(A));
u = [0 1]*expm(-transpose(A)*t(i))*inv(Rg)*[3;0];
%f = A*x+[0 0;0 1]*expm(-transpose(A)*t(i))*inv(expm(A*3)*[0 0;0 1]*expm(transpose(A)*3)-[0 0;0 1])*(expm(A*3)*x0);
f = A*x+[0;1]*u;%+ [0 0;0 1]*expm(transpose(A)*(2-t(i)))*inv(expm(A*2)*[0 0;0 1]*expm(transpose(A)*2)-[0 0;0 1])*(-expm(A*t(i))*x0);
shit(i) = ball.State(1);
ball.Force = [f(2)*15;0;0];
force(i) = f(2)*15;
ball.State = ball.homebrewRK4();
end
hold on
plot(t,shit,'g');
plot(2,0,'rh');
%figure(2)
%plot(t,force)
