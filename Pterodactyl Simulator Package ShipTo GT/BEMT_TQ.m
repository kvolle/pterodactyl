function [ FT,MT,InduVel] = BEMT_TQ( OmegaCMD, veltran, angrate, CGust, rho , h )
% Function File to Compute Thrust Forces and Torque Outputs
%   The outputs T and Q are of the form [1;2;3].

% Change Log
%{
    11/2012 - Function Written by Trevor Bennett
    5/15/2013 - Code Packaged by Trevor Bennett
%}

% Global Variables
global HubCoords ThrustLines TMap Control
global BladeR Bladex BladeCLAlpha BladeCD0 BladeCL0
global BladeN BladePitch BladeChord BladeDir
persistent k

if (isempty(k))
    k = 1;
end

ind = length(Bladex);

% Allocate Storage 
T = zeros(3,1);
Q = zeros(3,1);
InduVel = zeros(ind,3);

for i = 1:3
    % Velocity From Rotation at Hub [ft/s]
    d = HubCoords(i,1:3)';
    velrot = [(angrate(2)*d(3)-angrate(3)*d(2));
              (angrate(3)*d(1)-angrate(1)*d(3));
              (angrate(1)*d(2)-angrate(2)*d(1))];
        
    % Velocity at the Hub [ft/s]
    flowvel = CGust - veltran - velrot; 
    
    % Perpendicular Velocity Rel. to Prop Disk [ft/s]
    Vclimb = -(flowvel(1,1)*ThrustLines(i,1) + flowvel(2,1)*ThrustLines(i,2) + flowvel(3,1)*ThrustLines(i,3));
    
    % Assuming Constant Vclimb for all Annuli of a Propeller
    Omega = OmegaCMD(i);
    lambdac = Vclimb/(Omega*BladeR);
    
    % Zero Out Integration Indicies and Temp
    Ttemp = 0;
    Qtemp = 0;
    T1 = [0,0];             % [x coord, T @ x]
    Q1 = [0,0];             % [x coord, Q @ x]
    
    for j = 1:ind
        % Position Data
        x = Bladex(j);
        r = BladeR*x;
        c = BladeChord(j);
        Cla = BladeCLAlpha(j);
        Cd0 = BladeCD0(j);
        theta = BladePitch(j);
        
        % Compute Lambda
        sigma = BladeN*c/(pi*BladeR);
        
        lambda = -(sigma*Cla/16 - lambdac/2) + ...
            sqrt((sigma*Cla/16 - lambdac/2)^2 + sigma*Cla*theta*x/8);
        
        % Compute and Store Induced Velocity
        w = Omega*BladeR*(lambda - lambdac);
        InduVel(j,i) = w;   
        % Numerical Integration for T and Q
        vterm = Vclimb + w;
        rterm = Omega*r;
        Cl = Cla*(theta - vterm/rterm) + BladeCL0;
        
        T2(1,1) = x;
        T2(1,2) = c*rterm*(rterm*Cl - vterm*Cd0);
        Ttemp = Ttemp + BladeR*(x - T1(1,1))*(T1(1,2) + T2(1,2))/2;
        T1 = T2;
        
        Q2(1,1) = x;
        Q2(1,2) = c*rterm*(rterm*r*Cd0 + vterm*Cl*r);
        Qtemp = Qtemp + BladeR*(x - Q1(1,1))*(Q1(1,2) + Q2(1,2))/2;
        Q1 = Q2;
    end
    % Compute Integral and Store
    T(i,1) = BladeN*rho*Ttemp/2;
    Q(i,1) = BladeN*rho*Qtemp/2;
    
end

% Thrust Force Contributions
FT = ThrustLines'*T;

% Thrust Moment Contributions
MTT = TMap*T;
MQ = Q(1,1)*(-BladeDir(1))*ThrustLines(1,:)' + ...
    Q(2,1)*(-BladeDir(2))*ThrustLines(2,:)' + ...
    Q(3,1)*(-BladeDir(3))*ThrustLines(3,:)';
MT = MTT + MQ;

% Save Data
if (h == 0)
    Control.Thrust(k,1:3) = T;
    Control.BodyForce(k,1:3) = FT;
    Control.Torque(k,1:3) = Q;
    Control.BodyTorque(k,1:3) = MQ;
    k = k + 1;
end
    

end

