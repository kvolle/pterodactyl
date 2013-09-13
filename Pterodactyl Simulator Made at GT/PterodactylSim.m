clear all
clc
State = [0;0;-100;1;0;0;0;58.7;0;-3.91;0;0;0];
% State Vector: Xe, Ye, Ze, QW, QX, QY, QZ, u, v, w, p,q,r
%  This is strange but setting inertial reference frame parallel to body initially
%  So altitude is negative
Ib = [45239.31  -43.96  -91.58;...
      -43.96    991.74  0.90;...
      -91.58    0.90    54968.33];
m = 46.18;
Force = [0;0;0];
Moment = [0;0;0];


ball = quatRK4(State,Ib,m,Force,Moment);
test = zeros(2500,1);
test2 = zeros(2500,1);

tic

for i = 1:2500
    clc
    i
[htm,velocity,alpha,lift,drag,pitchMoment,area]= AerodynamicCoefficients(ball.State);


[aeroForces, aeroMoments] = resolveForces(htm,lift,drag,pitchMoment);

ball.Force = threshold(aeroForces)+[0;0;0];
ball.Moment = threshold(aeroMoments); %[0;0;0];%
tmp(i) = ball.State(8);

test(i) = ball.State(1);
test2(i) =  ball.State(3);
ball.State = ball.homebrewRK4();
if (mod(i,10) == 0)
    ball.State(4:7) = ball.State(4:7)*(1/norm(ball.State(4:7)));
end
    

al(i) = mean(alpha);
mom(i) = aeroMoments(2);

end
toc/i
plot(test,-test2,'gh');
hold off
%plot(al,'rh');