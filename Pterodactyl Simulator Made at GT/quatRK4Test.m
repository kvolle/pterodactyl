clear all
clc
State = [0;0;-10;1;0;0;0;0;0;0;0;0;0];
% State Vector: Xe, Ye, Ze, QW, QX, QY, QZ, u, v, w, p,q,r
Ib = eye(3);
m = 25;
Force = [0;0;0];
Moment = [0;0;0];


ball = quatRK4(State,Ib,m,Force,Moment);
test = zeros(15,1);
test2 = zeros(15,1);
for i = 1:15
ball.State = ball.homebrewRK4();
test(i) = ball.State(1);
test2(i) = ball.State(3);
end
plot(test,-test2,'rh')
hold on

%[htm,velocity,alpha,lift,drag,area]= AerodynamicCoefficients(ball.State);
%[aeroForces, aeroMoments] = resolveForces(htm,lift,drag)