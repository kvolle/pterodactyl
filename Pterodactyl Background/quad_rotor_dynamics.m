%Define System Parameters
%physical parameters of the sbRIO and aluminum plate
M = 0.292; %kg
W = 0.145;%m
H = 0.021;
L = 0.21;
%Rod
m = 0.256;
l = 0.5;

m_tot = M+4*m; %total weight
g = 9.81;
l_prop = 0.32; %length between COM of quadcop and prop

%Inertia Matrix Calculation
%inertia matrix for sbRIO / plate
Ixx_p = M*(W^2/12 + H^2/12);
Iyy_p = M*(L^2/12 + H^2/12);
Izz_p = M*(L^12/12 + W^2/12);

%inertia matrix of the arms / motors / rotors
Ixx_m = 0;
Iyy_m = m*l_prop^2;
Izz_m = m*l_prop^2;

%total inertia matrix
Ix = Ixx_p + Iyy_m + Izz_m;  
Iy = Iyy_p + Iyy_m + Izz_m;  
Iz = Izz_p + 4 * Izz_m;    

%calculate omega squared to simplify later calculations
omega1sq = omega1^2;
omega2sq = omega2^2;
omega3sq = omega3^2;
omega4sq = omega4^2;

%Input Equations
%omegas are propeller speed
% - note that g is in these equations only because it was a constant in the way the lift and drag were calculated
U1 = g*b*(omega1sq+omega2sq+omega3sq+omega4sq);
U2 = g*l_prop*b*(-omega2sq+omega4sq);
U3 = g*l_prop*b*(-omega1sq+omega3sq);
U4 = g*d*(-omega1sq+omega2sq-omega3sq+omega4sq)

%Trig computations - to simplify later math
stheta = sin(theta);
ctheta = cos(theta);
sphi = sin(phi);
cphi = cos(phi);

%Equations of Motion
% - these are in "aerospace" language
% u => x    p => thetax_dot   
% v => y    q => thetay_dot
% w => z    r => thetaz_dot
udot = (v*r - w*q) + g*stheta;
vdot = (w*p - u*r) - g*ctheta*sphi;
wdot = (u*q - v*p) - g*ctheta*cphi + U1/m_tot;

pdot = (Iy - Iz)/Ix * q * r + U2/Ix;
qdot = (Iz - Ix)/Iy * p * r + U3/Iy;
rdot = (Ix - Iy)/Iz * p * q + U4/Iz;