clear all
clc
State = [0;0;0;0;0;0;0;0;0;0;0;0;0];
% State Vector: Xe, Ye, Ze, QW, QX, QY, QZ, u, v, w, p,q,r
Ib = eye(3);
m = 2;
Force = [0;0;1];
Moment = [0;0;0];


ball = quatRK4(State,Ib,m,Force,Moment);
test = zeros(2000,1);
for i = 1:2000
ball.State = ball.homebrewRK4();
test(i) = ball.State(3);
end
plot(test)