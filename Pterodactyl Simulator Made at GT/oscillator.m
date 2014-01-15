clear all
clc

x = [3;0];

A = [0 1;-2 -3];
hold on
state = [-2;0;0;0;0;0;0;0;0;0;0;0];
Ib = [263 0 0;0 115 0;0 0 331];
m = 1;
Force = [0;0;0];
Moment = [0;0;0];

ball = eulerRK4(state,Ib,m,Force,Moment);

for i = 1:5000
x = [ball.State(1);ball.State(7)];
%plot(i/1000,x(1),'b');
%test(i) = x(1);

t(i) = i/1000;
%f = (A-[0 0;1050 1050])*x;
f = [0 1;-5 -6]*x;%+[0;15];
test(i) = x(1);
%g = A*[ball.State(6);ball.State(12)];
%a(i) = f(2);
ball.Force = [f(2);0;0];
ball.State = ball.homebrewRK4();
end
hold on
plot(t,test,'r');
