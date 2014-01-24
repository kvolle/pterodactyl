function torque = torqueCalculator(thrust);

rho = 0.076474; % lbm/ft^3
chord = 0.1875; % feet
r = 0.75; % feet
h = 20;
B = 3;
dr = r/h;
theta = 5*pi/180;
Cl = 6.2796*theta;
Cd = 0.1604*theta;


for i = 1:h

dL(i) = 0.5*Cl*rho*chord*dr*((i-0.5)*dr)^2;
dD(i) = 0.5*Cd*rho*chord*dr*((i-0.5)*dr)^2;
dT(i) = dL(i)*cos(theta)-dD(i)*sin(theta);
dTau(i) = (i-0.5)*dr*(dL(i)*sin(theta)+dD(i)*cos(theta));
end

omega = sqrt(12*sum(dT)*thrust)/(6*sum(dT))

torque = 3*sum(dTau)*omega^2;