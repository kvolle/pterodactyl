clear all
clc

State = [0;0;0;0; pi/2;0;0;0;0;0;0;0];
Ib = eye(3);
m = 1;
Force = [0;0;0];
Moment = [0;0;0];
ball = eulerRK4(State,Ib,m,Force,Moment);

for i = 1:1000
    ball.State = ball.homebrewRK4();
    plot(i,ball.State(7));
    hold on
end
