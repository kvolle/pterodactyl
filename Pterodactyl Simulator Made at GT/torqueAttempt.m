clear all
clc

radius = 2/3; %feet
dr = radius/20;
B = 2; % number of blades
theta = 15*pi/180; %radians
alpha = ones(1,20)*122*pi/180; %radians, just a guess
phi = theta-alpha;
rho = 0.076474; % lbm/ft^3
omega = 60; % rad/s ????
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
v_in = 1; %feet/sec velocity normal to actuator disk
v1 = ones(1,20)*15; %starting guess
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
chord = 0.0208;
Cl = 0.55;
Cd = 0.014;


hold on
dT = zeros(1,20);
dQ = zeros(1,20);
a = zeros(1,20);
b = zeros(1,20);
v0 = zeros(1,20);
v2 = zeros(1,20);

for x = 1:50
  for i = 1:20
      dT(i) = 0.5*rho*chord*(Cl*cos(phi(i))-Cd*sin(phi(i)))*B*dr*v1(i)^2;
      dQ(i) = (i-0.5)*dr*0.5*rho*chord*(Cl*sin(phi(i))+Cd*cos(phi(i)))*B*dr*v1(i)^2;
      
      
    
  end
  plot(x,sum(dQ),'r.');
end  
%sum(dT)
%sum(dQ)