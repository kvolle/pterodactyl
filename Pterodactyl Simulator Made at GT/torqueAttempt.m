clear all
clc

radius = 8; %inches
dr = radius/20;
B = 2; % number of blades
theta = 5*pi/180; %radians
alpha = 2*pi/180; %radians, just a guess
phi = theta-alpha;
rho = 0.076474; % lbm/ft^3
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
v = 15; %feet/sec velocity normal to actuator disk
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
chord = 0.25;
Cl = 0.55;
Cd = 0.014;
for i = 1:20
    dT(i) = 0.5*rho*(Cl*cos(phi) - Cd*sin(phi))*chord*dr*B*v^2;
    dQ(i) = 0.5*rho*(Cl*sin(phi) + Cd*cos(phi))*chord*dr*B*dr*(i-0.5)*v^2;
end
sum(dT)
sum(dQ)