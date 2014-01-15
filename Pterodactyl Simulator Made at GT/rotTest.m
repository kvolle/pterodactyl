clear all
clc
 a = random('unif',0,200)/100 -1;

State = [0;0;-100;0;0;a;0;0;0;0;0;0];
% State Vector: Xe, Ye, Ze, QW, QX, QY, QZ, u, v, w, p,q,r
%  This is strange but setting inertial reference frame parallel to body initially
%  So altitude is negative
Ib = [1 0 0;0 1 0;0 0 1];
m = 46.18;
Force = [0;0;0];
Moment = [0;0;0];


ball = eulerRK4(State,Ib,m,Force,Moment);
test = zeros(2500,1);
test2 = zeros(2500,1);
A = [0 1;-2 -3];
for i = 1:5000;
    x =[ball.State(6);ball.State(12)];
    f = A*x
    test(i) = ball.State(6);
    ball.Moment = [0;0;f(2)];
    ball.State = ball.homebrewRK4();
end
plot(test);