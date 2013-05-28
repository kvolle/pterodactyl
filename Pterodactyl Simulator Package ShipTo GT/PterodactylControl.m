function [ CMD ] = PterodactylControl(yn)
% Function to Compute the Commanded Motor Angular Velocities and the
% Control Surface Deflections
%   This is a temporary place holder for future control algorithms.
% The controller is not aware of gusts.

% Change Log
%{
    3/2012 - Function Written by Trevor Bennett
    5/15/2013 - Code Packaged by Trevor Bennett
%}

global I m g TMapInv TLMapInv ThrustLines HubCoords MaxMAR

% Unpack States
z = yn(3,1);
B0 = yn(4,1);
B1 = yn(5,1);
B2 = yn(6,1);
B3 = yn(7,1);
Quat = [B0; B1; B2; B3];
u = yn(8,1);
v = yn(9,1);
w = yn(10,1);
p = yn(11,1);
q = yn(12,1);
r = yn(13,1);

% Translation and Rotation Velocities
veltran = [u;v;w];                  % Velocity from Translation
angrate = [p;q;r];                  % Rotational Angular Velocity

% Atmospheric Estimate - Density 
if (z > -35332)
    rho = 0.0023784722 * (1.0 + 6.8789e-6*z)^(4.258);
else
    rho = 7.2674385e-4 * exp(4.78e-5*(z+35332));
end


%% First Pass Controller %%
% Conceived by Dr. Rogers
% Inertia Formulations
Ixx = I(1,1);
Iyy = I(2,2);
Izz = I(3,3);
Ixz = I(1,3);
Ibar = Ixz^2 - Ixx*Izz;
wIw = [0 -r q;r 0 -p;-q p 0]*[Ixx 0 Ixz;0 Iyy 0;Ixz 0 Izz]*[p;q;r];

% Control Gains
k1 = 10;
k2 = 10;
k3 = 10;

% Commanded Rates (Outer Loop)
pc = 0;
qc = 0;
rc = 0;
pcdot = 0;
qcdot = 0;
rcdot = 0;

% Control Errors
pe = p - pc;
qe = q - qc;
re = r - rc;

% Desired Moments
desiredM = [(Ixx*(-k1*pe + Ixz*wIw(3)/Ibar - Izz*wIw(1)/Ibar + pcdot) + Ixz*(-k3*re + Ixz*wIw(1)/Ibar - Ixx*wIw(3)/Ibar +rcdot))/(Ibar^2);
            wIw(2) - k2*qe + qcdot;
            (Ixz*(-k1*pe + Ixz*wIw(3)/Ibar - Izz*wIw(1)/Ibar + pcdot) + Izz*(-k3*re + Ixz*wIw(1)/Ibar - Ixx*wIw(3)/Ibar +rcdot))/(Ibar^2)];
        
        
% Thrust Desired
desiredT = TMapInv*desiredM;

%% Hover Control %%
% TBD

%% Force and Moment to Control Mapping %%
% [F_des;M_des] = [6x6]*[T_req;daoa_controls] + MQ
global ControlSurface RotL RotR CFxCoeff CFzCoeff CmCoeff BladeDir TMap

% Desired Forces and Moments from Control Law
F_des = [0;0;0];
M_des = [0;0;0];

% Estimated Dynamic Pressure for Control Surfaces
d = ControlSurface.L.r';
velrot = [(angrate(2)*d(3)-angrate(3)*d(2));
          (angrate(3)*d(1)-angrate(1)*d(3));
          (angrate(1)*d(2)-angrate(2)*d(1))];
V = - veltran - velrot;
q_l = 0.5*rho*dot(V,V);
d = ControlSurface.C.r';
velrot = [(angrate(2)*d(3)-angrate(3)*d(2));
          (angrate(3)*d(1)-angrate(1)*d(3));
          (angrate(1)*d(2)-angrate(2)*d(1))];
V = - veltran - velrot;
q_c = 0.5*rho*dot(V,V);
d = ControlSurface.R.r';
velrot = [(angrate(2)*d(3)-angrate(3)*d(2));
          (angrate(3)*d(1)-angrate(1)*d(3));
          (angrate(1)*d(2)-angrate(2)*d(1))];
V = - veltran - velrot;
q_r = 0.5*rho*dot(V,V);

%%% Build 6x6 Matrix %%%
coeVec1 = [CFxCoeff;0;CFzCoeff];
UR = [ControlSurface.L.Sc*q_l*RotL'*coeVec1,ControlSurface.C.Sc*q_c*coeVec1,ControlSurface.R.Sc*q_r*RotR'*coeVec1];

coeVec2 = [0;CmCoeff;0];
LRp1 = [cross(ControlSurface.L.r',UR(1:3,1)),cross(ControlSurface.C.r',UR(1:3,2)),cross(ControlSurface.R.r',UR(1:3,3))];
LRp2 = [ControlSurface.L.Sc*ControlSurface.L.avc*q_l*RotL'*coeVec2,ControlSurface.C.Sc*ControlSurface.C.avc*q_c*coeVec2,ControlSurface.R.Sc*ControlSurface.L.avc*q_r*RotR'*coeVec2];
LR = LRp1 + LRp2;

% Thrust Moment Contributions
Qc = 0.0675;                % Approximation
MQ = Qc*[(-BladeDir(1))*ThrustLines(1,:)',(-BladeDir(2))*ThrustLines(2,:)',(-BladeDir(3))*ThrustLines(3,:)'];

% 6x6 Matrix
ControlMap = [ThrustLines',UR;TMap,LR];

%%% Invert 6x6 onto Desired Forces and Moments %%%
% -> Perhaps use a weighted inverse to weight control
% Temp solution
temp = [30;3;3;0.01;0;0.01];

% Extract Controls
T_req = temp(1:3,1);
daoa_l = temp(4,1);
daoa_e = temp(5,1);
daoa_r = temp(6,1);

%% Commanded Thrust and Motor Angular Rate (MAR) %%
T_CMD = T_req;

CMD.Omega = [0;0;0];
for i = 1:3
    % Velocity From Rotation at Hub [ft/s]
    d = HubCoords(i,1:3)';
    velrot = [(angrate(2)*d(3)-angrate(3)*d(2));
              (angrate(3)*d(1)-angrate(1)*d(3));
              (angrate(1)*d(2)-angrate(2)*d(1))];
        
    % Estimated Velocity at the Hub [ft/s]
    flowvel = - veltran - velrot; 
    
    % Perpendicular Velocity Rel. to Prop Disk [ft/s]
    vClimb = -(flowvel(1,1)*ThrustLines(i,1) + flowvel(2,1)*ThrustLines(i,2) + flowvel(3,1)*ThrustLines(i,3));
    
    % Compute Commanded Motor Angular Rate (MAR)
    CMD.Omega(i,1) = OmegaCalc(T_CMD(i,1),vClimb);
end 

% NOT GOOD PRACTICE!! MUST FIX
CMD.Omega = real(CMD.Omega);

% Consider Motor Saturation
for i = 1:3
    if (abs(CMD.Omega(i,1)) >= MaxMAR)
        CMD.Omega(i,1) = MaxMAR*sign(CMD.Omega(i,1));
    end
end

%% Control Surface Deflections
%{
The Control Surfaces do not follow aircraft conventions. By convention, 
positive deflection represents a positive moment about the body axis. 
The deflection angles for control surfaces in this code is that a positive 
deflection represents a positive change in angle of attack for the
airfoil in the airfoil fixed frame.
%}
syms da de
% Compute Left Aileron Deflection
asol_l = real(double(solve(tan(daoa_l)*(ControlSurface.L.lhinge + ControlSurface.L.xbar*cos(da)) - ControlSurface.L.xbar*sin(da),da)));
if length(asol_l) == 1
    CMD.da_l = asol_l(1);
elseif abs(asol_l(1)) <= abs(asol_l(2))
    CMD.da_l = asol_l(1);
else
    CMD.da_l = asol_l(2);
end

% Compute Elevator Deflection
asol_e = real(double(solve(tan(daoa_e)*(ControlSurface.C.chinge + ControlSurface.C.xbar*cos(de)) - ControlSurface.C.xbar*sin(de),de)));
if length(asol_e) == 1
    CMD.de = asol_e(1);
elseif abs(asol_e(1)) <= abs(asol_e(2))
    CMD.de = asol_e(1);
else
    CMD.de = asol_e(2);
end

% Compute Right Aileron Deflection
asol_r = real(double(solve(tan(daoa_r)*(ControlSurface.R.rhinge + ControlSurface.R.xbar*cos(da)) - ControlSurface.R.xbar*sin(da),da)));
if length(asol_r) == 1
    CMD.da_r = asol_r(1);
elseif abs(asol_r(1)) <= abs(asol_r(2))
    CMD.da_r = asol_r(1);
else
    CMD.da_r = asol_r(2);
end


CMD.da_l = 0.0873;       % Left Aileron Deflection
CMD.de = 0;         % Elevator Deflection
CMD.da_r = 0.0873;       % Right Aileron Deflection
end

