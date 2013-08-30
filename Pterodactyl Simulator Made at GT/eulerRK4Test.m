clear all
clc
State = [0;0;0;0;0;0;0;0;0;0;0;0];
Ib = eye(3);
m = 2;
Force = [0;0;1];
Moment = [0;0;0];

ball = eulerRK4(State,Ib,m,Force,Moment);

test = [10;10;10;10;0;0;0;0;0;0];
for i = 1:10

ball.State = ball.homebrewRK4();
test(i) = ball.State(3);
end
plot(test)