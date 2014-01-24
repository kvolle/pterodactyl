clear all
clc

m = 15;
Force = [0;0;0];
Moment = [0;0;0];
State = [0;0;-8;-pi/2;0;0;0;0;0;0;0;0];
Ib = eye(3);

ball = eulerRK4(State,Ib,m,Force,Moment);
n = 10000;
test1 = zeros(1,n);
test2 = zeros(1,n);
test3 = zeros(1,n);

error = 0;
for i = 1:n
    Or = ball.State(4:6);
    R = [cos(Or(2))*cos(Or(3)) -cos(Or(2))*sin(Or(3)) sin(Or(2));...
         cos(Or(1))*sin(Or(3))+cos(Or(3))*sin(Or(1))*sin(Or(2)) cos(Or(1))*cos(Or(3))-sin(Or(1))*sin(Or(2))*sin(Or(3)) -cos(Or(2))*sin(Or(1));...
         sin(Or(1))*sin(Or(3))-cos(Or(1))*cos(Or(3))*sin(Or(2)) cos(Or(3))*sin(Or(1))+cos(Or(1))*sin(Or(2))*sin(Or(3)) cos(Or(1))*cos(Or(2))];
    a = ball.State(1:3);
    
    tmp = R*ball.State(7:9)
    error = error + (ball.State(3)+10)/1000
    acc = -15*(ball.State(3)+10) -20*tmp(3) -10*error;
    ball.Force = R\(m*[0;0;acc]);
    
    test2(i) = ball.State(3);
    test1(i) = tmp(3);

    ball.State = ball.homebrewRK4();
end
%{
figure(1)
hold on
plot(test1,'k');
plot(test3,'g');
figure(2)
plot(test2);
%}

plot(test3,'k');
hold on
plot(-test2,'g');
