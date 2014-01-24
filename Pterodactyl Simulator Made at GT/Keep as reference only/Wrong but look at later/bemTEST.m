function [omega,torque] = bemTEST(thrust)
%clear all
%clc

%
r = 0.75; % Feet
chord = 0.0417; % Feet (half inch)
B = 3; % Number of blades
sigma = B*chord/(pi*r); % Actuator disk solidity
theta = 5*pi/180; % radians
%Cl = 0.10966; %2pi/rad converted to deg
Cl = 2*pi;
rho = 0.076474; %lbm/ft^3
theta_d = 5; % degrees
omega = 600; % rad/s
Cd = 0.40107;
%}

%{
% Chinook reality check
r = 30; % Feet
chord = 1; % Feet (half inch)
B = 3; % Number of blades
sigma = B*chord/(pi*r); % Actuator disk solidity
theta = 5*pi/180; % radians
Cl = 0.10966; %2pi/deg converted to radains
Cl_d = 2*pi;
rho = 0.076474; %lbm/ft^3
theta_d = 5; % degrees
omega = 225; % rad/s
%}

size = 50;
tmp = zeros(1,size);
tmp2 = zeros(1,size);
lambda = .005; % Initial guess

i =2;
while((lambda-tmp2(i-1))/lambda >0.01)
    
ct = sigma*Cl*(theta/6 - lambda/4);
tmp(i) = ct;
tmp2(i) = lambda;
lambda = sqrt(ct/2);
i = i+1;
end

%plot(tmp,'b');
%hold on
%plot(tmp2,'k');
ct;
cq =  sigma*0.014/8 + sigma*lambda*Cl*(theta/6 -lambda/4);
sigma*0.1604*theta/8;
%thrust = ct*rho*(pi*r^2)*((omega*r)^2)
omega = sqrt(thrust/(ct*rho*pi*r^4));
%thrust = ct*rho*(pi*r^2)*((omega*r)^2)
torque = cq*rho*(pi*r^2)*((omega*r)^2)*r;
