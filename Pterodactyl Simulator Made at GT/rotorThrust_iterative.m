clear all
clc

omega = 500; % RPM
R = 0.75; % feet
A = pi*R^2; % square feet
theta = 5*pi/180; %blade pitch angle
Cl = 5.7; % rough guess

sigma = 2*0.1/(pi*R); % guess on chord length
lambda = 0.15; % initial guess, << 1

change = 1;
i = 0;
while ((change > 0.01)&&(i<100))
   i = i+1
    Ct = sigma*Cl*(theta/6 -lambda/4);
    new_lambda = sqrt(Ct/2);
    change = (new_lambda-lambda)/lambda;
    lambda = new_lambda;
end
