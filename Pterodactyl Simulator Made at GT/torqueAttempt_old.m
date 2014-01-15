clear all
clc

radius = 2/3; %feet
dr = radius/20;
B = 2; % number of blades
theta = 5*pi/180; %radians
alpha = ones(1,20)*2*pi/180; %radians, just a guess
phi = theta-alpha;
rho = 0.076474; % lbm/ft^3
omega = 60; % rad/s ????
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
v_in = 0; %feet/sec velocity normal to actuator disk
v1 = ones(1,20)*15; %starting guess
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
chord = 0.25;
Cl = 0.55;
Cd = 0.014;

hold on

for x = 1:50
  for i = 1:20
    phi(i) = theta-alpha(i)
    dT(i) = 0.5*rho*(Cl*cos(phi(i)) - Cd*sin(phi(i)))*chord*dr*B*v1(i)^2;
    dQ(i) = 0.5*rho*(Cl*sin(phi(i)) + Cd*cos(phi(i)))*chord*dr*B*dr*(i-0.5)*v1(i)^2;
    a(i) = dT(i)/(rho*4*pi*(i-0.5)*dr*v_in^2) - 1;
    b(i) = dQ(i)/(rho*4*pi*(((i-0.5)*dr)^3)*omega*dr*(1+a(i))*v_in^2)
    v0(i) = v_in*(1+a(i));
    v2(i) = omega*(i-0.5)*dr*(1-b(i));
    v1(i) = sqrt(v0(i)^2 + v2(i)^2);
    alpha(i) = theta-tan(v0(i)/v2(i));
  end
  plot(x,sum(dQ),'r.');
end  
%sum(dT)
%sum(dQ)