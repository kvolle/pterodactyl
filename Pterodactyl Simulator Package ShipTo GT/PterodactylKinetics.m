function [fofy] = PterodactylKinetics(h,yn,CMD)
% Function to Generate Matrix of Dynamic Differential Equations
%   The variables passed in are T and y. 
%       h represents the variation in the current time. Passed in from the
%       Runge Kutta to be used in DrydenGust.
%       y represents the Aircraft State Vector:
%           [x; y; z; Beta0; Beta1; Beta2; Beta3; u; v; w; p; q; r]

% Change Log
%{
    10/2012 - Function Written by Trevor Bennett
    5/2013 - Aileron Effetiveness Included by Trevor Bennett
    5/15/2013 - Code Packaged by Trevor Bennett
%}

%% Glogal Variables %%
% General
global g m  I Iinv b wn optionsDG
% Geometry
global CStripCoords RStripCoords LStripCoords RotR RotL
global cnstrip rnstrip lnstrip
global cchords rchords lchords
global CStripArea RStripArea LStripArea
global NegTriCent NegTriArea NegTriNormal
% Aerodynamics
global mu Alpha ReCF CFxAlpha CFzAlpha ReCm CmAlpha MeshMult ControlSurface
% Propulsion
global ThrustLines LPropEffect CPropEffect RPropEffect Bladex

persistent errflg
if isempty(errflg)
    errflg = 0;
end

%% Initialize %%
fofy = zeros(13,1);

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

% Check for Ground Contact
if (z >= 0)
    if (errflg == 0)
        disp('The Pterodactyl has touched the ground. The simualtion is no longer accurate')
        errflg = 1;
        return;
    else
        return;
    end
end

% Rotatioon Matrix and Euler Angles
C = ctm_chck(quat2ctm(Quat));

% Gravity
Grav = C*[0;0;m*g];
XGrav = Grav(1);
YGrav = Grav(2);
ZGrav = Grav(3);
 
%% Gust Model %%
% Dryden Gust
[gusttotal] = DrydenGust([u,v,w],z,b,wn,C,h,optionsDG);

% Extract the Body Frame Directional Wind Profile Components
CGust = gusttotal(1:3,1);

% Density 
if (z > -35332)
    rho = 0.0023784722 * (1.0 + 6.8789e-6*z)^(4.258);
else
    rho = 7.2674385e-4 * exp(4.78e-5*(z+35332));
end

% Atmospheric Pressure
% height = -z*0.3048;                                 % In meters
% pp = 101325*(1 - (2.25577E-5)*height)^5.25588;      % Pressure [Pa]
% p0 = pp*0.000145038*144;                            % Pressure [lb/ft^2]

%% Propulsive Influence %%
% Thrust and Torques of Propellors
veltran = [u;v;w];                  % Velocity from Translation
angrate = [p;q;r];                  % Rotational Angular Velocity
[FT,MT,InduVel] = BEMT_TQ(CMD.Omega, veltran, angrate, CGust, rho, h);

%% Forces And Moments %%
% Zero Out Forces and Moments
FR = zeros(3,1);
FL = zeros(3,1);
FC = zeros(3,1);
MR = zeros(3,1);
ML = zeros(3,1);
MC = zeros(3,1);
% Left Wing Forces and Moments 
for i = 1:lnstrip
    d = LStripCoords(i,:)';
    % Velocity from Rotation
    velrot = [(angrate(2)*d(3)-angrate(3)*d(2));
              (angrate(3)*d(1)-angrate(1)*d(3));
              (angrate(1)*d(2)-angrate(2)*d(1))];
    
    % Sum of Local Air Velocities in Left Wing Frame
    if (LPropEffect(i,1) == 1)
        flag = 0;
        k = 1;
        while (flag == 0)
            if (Bladex(k+1) >= LPropEffect(i,2))
                indu = InduVel(k,1) + (LPropEffect(i,2)-Bladex(k))*(InduVel(k+1,1)-InduVel(k,1))/(Bladex(k+1)-Bladex(k));
                induced = (ThrustLines(1,:)')*2*indu;
                flag = 1;
            end
            k = k + 1;
        end
        if (norm(+isnan(induced)) >= 1)
            induced = [0;0;0];
        end
        flowvel = RotL*(CGust - veltran - velrot - induced);
    else
        flowvel = RotL*(CGust - veltran - velrot);
    end
    
    if (isreal(flowvel) == 0)
        disp(flowvel)
    end
        
    % Velocity in Airfoil Frame
    velsq = flowvel(1)^2 + flowvel(3)^2;
    dynpres = 0.5*rho*velsq;                % Dynamic Pressure
    Re = rho*sqrt(velsq)*lchords(i,1)/mu;   % Airfoil Re Number       
    
    % Angle of Attack
    if (ControlSurface.L.L(i,1) == 1)
        daoa = atan2(ControlSurface.L.L(i,2)*sin(CMD.da_l),(ControlSurface.L.lhinge + ControlSurface.L.L(i,2)*cos(CMD.da_l)));
        aoaflow = atan2(-flowvel(3),-flowvel(1));  % [rad]
        aoarad = aoaflow + daoa;
    else
        aoarad = atan2(-flowvel(3),-flowvel(1));  % [rad]
    end
    if (aoarad >= 0)
        aoa = aoarad*180/pi;
    else
        aoa = 360 + aoarad*180/pi;
    end
    if (aoa >= 360)
        aoa = aoa - 360;
    end
    
    % Coefficient Extraction
    aoascale = round(aoa*MeshMult);
    aoaind = find(Alpha == aoascale);       % The Alpha Vector is Scaled
    flag = 0;
    if (Re < ReCF(1))
        cfx = CFxAlpha(aoaind,1);
        cfz = CFzAlpha(aoaind,1);
    elseif (Re <= ReCF(end))
        k = 1;
        while (flag == 0)
            if (ReCF(k+1) >= Re)
                cfx = CFxAlpha(aoaind,k) + (Re-ReCF(k))*(CFxAlpha(aoaind,k+1)-CFxAlpha(aoaind,k))/(ReCF(k+1)-ReCF(k));
                cfz = CFzAlpha(aoaind,k) + (Re-ReCF(k))*(CFzAlpha(aoaind,k+1)-CFzAlpha(aoaind,k))/(ReCF(k+1)-ReCF(k));
                flag = 1;
            end
            k = k + 1;
        end
    else
        cfx = CFxAlpha(aoaind,end);
        cfz = CFzAlpha(aoaind,end);
    end
    flag = 0;
    if (Re < ReCm(1))
        cm = CmAlpha(aoaind,1);
    elseif (Re <= ReCm(end))
        k = 1;
        while (flag == 0)
            if (ReCm(k+1) >= Re)
                cm = CmAlpha(aoaind,k) + (Re-ReCm(k))*(CmAlpha(aoaind,k+1)-CmAlpha(aoaind,k))/(ReCm(k+1)-ReCm(k));
                flag = 1;
            end
            k = k + 1;
        end
    else
        cm = CmAlpha(aoaind,end);
    end
    
    % Sum Forces and Moments
    
    f = dynpres*LStripArea(i)*(RotL'*[cfx;0;cfz]);
    FL = FL + f;
    mo = dynpres*LStripArea(i)*lchords(i,1)*(RotL'*[0;cm;0]);
    ML = ML + mo + [(d(2)*f(3)-d(3)*f(2));(d(3)*f(1)-d(1)*f(3));(d(1)*f(2)-d(2)*f(1))];
end

% Center Wing Forces and Moments
for i = 1:cnstrip
    d = CStripCoords(i,:)';
    
    % Velocity from Rotation
    if (i == (floor(cnstrip/2)+1))
        velrot = [0;0;0];
    else
        % Velocity from Rotation
        velrot = [(angrate(2)*d(3)-angrate(3)*d(2));
              (angrate(3)*d(1)-angrate(1)*d(3));
              (angrate(1)*d(2)-angrate(2)*d(1))];
    end
    
    % Sum of Local Air Velocities in Left Wing Frame
    if (CPropEffect(i,1) == 1)
        flag = 0;
        k = 1;
        while (flag == 0)
            if (Bladex(k+1) >= CPropEffect(i,2))
                indu = InduVel(k,2) + (CPropEffect(i,2)-Bladex(k))*(InduVel(k+1,2)-InduVel(k,2))/(Bladex(k+1)-Bladex(k));
                induced = (ThrustLines(2,:)')*2*indu;
                flag = 1;
            end
            k = k + 1;
        end
        if (norm(+isnan(induced)) >= 1)
            induced = [0;0;0];
        end
        flowvel = CGust - veltran - velrot - induced;
    else
        flowvel = CGust - veltran - velrot;
    end
    
    % Velocity in Airfoil Frame
    velsq = flowvel(1)^2 + flowvel(3)^2;
    dynpres = 0.5*rho*velsq;                % Dynamic Pressure
    Re = rho*sqrt(velsq)*cchords(i,1)/mu;   % Airfoil Re Number       
    
    % Angle of Attack
    if (ControlSurface.C.C(i,1) == 1)
        daoa = atan2(ControlSurface.C.C(i,2)*sin(CMD.de),(ControlSurface.C.chinge + ControlSurface.C.C(i,2)*cos(CMD.de)));
        aoaflow = atan2(-flowvel(3),-flowvel(1));  % [rad]
        aoarad = aoaflow + daoa;
    else
        aoarad = atan2(-flowvel(3),-flowvel(1));  % [rad]
    end
    if (aoarad >= 0)
        aoa = aoarad*180/pi;
    else
        aoa = 360 + aoarad*180/pi;
    end
    if (aoa == 360)
        aoa = 0;
    end
    
    % Coefficient Extraction
    aoascale = round(aoa*MeshMult);
    aoaind = find(Alpha == aoascale);
    flag = 0;
    if (Re < ReCF(1))
        cfx = CFxAlpha(aoaind,1);
        cfz = CFzAlpha(aoaind,1);
    elseif (Re <= ReCF(end))
        k = 1;
        while (flag == 0)
            if (ReCF(k+1) >= Re)
                cfx = CFxAlpha(aoaind,k) + (Re-ReCF(k))*(CFxAlpha(aoaind,k+1)-CFxAlpha(aoaind,k))/(ReCF(k+1)-ReCF(k));
                cfz = CFzAlpha(aoaind,k) + (Re-ReCF(k))*(CFzAlpha(aoaind,k+1)-CFzAlpha(aoaind,k))/(ReCF(k+1)-ReCF(k));
                flag = 1;
            end
            k = k + 1;
        end
    else
        cfx = CFxAlpha(aoaind,end);
        cfz = CFzAlpha(aoaind,end);
    end
    flag = 0;
    if (Re < ReCm(1))
        cm = CmAlpha(aoaind,1);
    elseif (Re <= ReCm(end))
        k = 1;
        while (flag == 0)
            if (ReCm(k+1) >= Re)
                cm = CmAlpha(aoaind,k) + (Re-ReCm(k))*(CmAlpha(aoaind,k+1)-CmAlpha(aoaind,k))/(ReCm(k+1)-ReCm(k));
                flag = 1;
            end
            k = k + 1;
        end
    else
        cm = CmAlpha(aoaind,end);
    end
    
    % Sum Forces and Moments
    f = dynpres*CStripArea(i)*[cfx;0;cfz];
    FC = FC + f;
    mo = dynpres*CStripArea(i)*cchords(i,1)*[0;cm;0];
    MC = MC + mo + [(d(2)*f(3)-d(3)*f(2));(d(3)*f(1)-d(1)*f(3));(d(1)*f(2)-d(2)*f(1))];
end

% Right Wing Forces and Moments
for i = 1:rnstrip
    d = RStripCoords(i,:)';
    % Velocity from Rotation
    velrot = [(angrate(2)*d(3)-angrate(3)*d(2));
              (angrate(3)*d(1)-angrate(1)*d(3));
              (angrate(1)*d(2)-angrate(2)*d(1))];
     
    % Sum of Local Air Velocities in Left Wing Frame
    if (RPropEffect(i,1) == 1)
        flag = 0;
        k = 1;
        while (flag == 0)
            if (Bladex(k+1) >= RPropEffect(i,2))
                indu = InduVel(k,3) + (RPropEffect(i,2)-Bladex(k))*(InduVel(k+1,3)-InduVel(k,3))/(Bladex(k+1)-Bladex(k));
                induced = (ThrustLines(3,:)')*2*indu;
                flag = 1;
            end
            k = k + 1;
        end
        if (norm(+isnan(induced)) >= 1)
            induced = [0;0;0];
        end
        flowvel = RotR*(CGust - veltran - velrot - induced);
    else
        flowvel = RotR*(CGust - veltran - velrot);
    end
    
    % Velocity in Airfoil Frame
    velsq = flowvel(1)^2 + flowvel(3)^2;
    dynpres = 0.5*rho*velsq;                % Dynamic Pressure
    Re = rho*sqrt(velsq)*rchords(i,1)/mu;   % Airfoil Re Number       
    
    % Angle of Attack
    if (ControlSurface.R.R(i,1) == 1)
        daoa = atan2(ControlSurface.R.R(i,2)*sin(CMD.da_r),(ControlSurface.R.rhinge + ControlSurface.R.R(i,2)*cos(CMD.da_r)));
        aoaflow = atan2(-flowvel(3),-flowvel(1));  % [rad]
        aoarad = aoaflow + daoa;
    else
        aoarad = atan2(-flowvel(3),-flowvel(1));  % [rad]
    end
    if (aoarad >= 0)
        aoa = aoarad*180/pi;
    else
        aoa = 360 + aoarad*180/pi;
    end
    if (aoa == 360)
        aoa = 0;
    end
    
    % Coefficient Extraction
    aoascale = round(aoa*MeshMult);
    aoaind = find(Alpha == aoascale);
    flag = 0;
    if (Re < ReCF(1))
        cfx = CFxAlpha(aoaind,1);
        cfz = CFzAlpha(aoaind,1);
    elseif (Re <= ReCF(end))
        k = 1;
        while (flag == 0)
            if (ReCF(k+1) >= Re)
                cfx = CFxAlpha(aoaind,k) + (Re-ReCF(k))*(CFxAlpha(aoaind,k+1)-CFxAlpha(aoaind,k))/(ReCF(k+1)-ReCF(k));
                cfz = CFzAlpha(aoaind,k) + (Re-ReCF(k))*(CFzAlpha(aoaind,k+1)-CFzAlpha(aoaind,k))/(ReCF(k+1)-ReCF(k));
                flag = 1;
            end
            k = k + 1;
        end
    else
        cfx = CFxAlpha(aoaind,end);
        cfz = CFzAlpha(aoaind,end);
    end
    flag = 0;
    if (Re < ReCm(1))
        cm = CmAlpha(aoaind,1);
    elseif (Re <= ReCm(end))
        k = 1;
        while (flag == 0)
            if (ReCm(k+1) >= Re)
                cm = CmAlpha(aoaind,k) + (Re-ReCm(k))*(CmAlpha(aoaind,k+1)-CmAlpha(aoaind,k))/(ReCm(k+1)-ReCm(k));
                flag = 1;
            end
            k = k + 1;
        end
    else
        cm = CmAlpha(aoaind,end);
    end
    
    % Sum Forces and Moments
    f = dynpres*RStripArea(i)*(RotR'*[cfx;0;cfz]);
    FR = FR + f;
    mo = dynpres*RStripArea(i)*rchords(i,1)*(RotR'*[0;cm;0]);
    MR = MR + mo + [(d(2)*f(3)-d(3)*f(2));(d(3)*f(1)-d(1)*f(3));(d(1)*f(2)-d(2)*f(1))];
end

% Triangular Plate Pressure Force
%  (Treated as a jet of air hitting a flat plate)
FNT = zeros(3,1);
MNT = zeros(3,1);
for i = 1:4
    d = NegTriCent(i,:)';
    % Velocity from Rotation
    velrot = [(angrate(2)*d(3)-angrate(3)*d(2));
              (angrate(3)*d(1)-angrate(1)*d(3));
              (angrate(1)*d(2)-angrate(2)*d(1))];
    
    % Sum of Local Air Velocities
    flowvel = CGust - veltran - velrot;
    velsq = (flowvel')*flowvel;
    
    if (velsq ~= 0)
        % Cosine of Angle Between Flow and Plate
        cang = dot(flowvel,NegTriNormal(i,:))/norm(flowvel);

        % Sum Forces and Moments
        f = (rho*NegTriArea(i,1)*velsq*cang)*NegTriNormal(i,:)';
        FNT = FNT + f;
        MNT = MNT + [(d(2)*f(3)-d(3)*f(2));(d(3)*f(1)-d(1)*f(3));(d(1)*f(2)-d(2)*f(1))];    
    end
end

% Force Equations
X = FR(1) + FL(1) + FC(1) + FNT(1) + XGrav + FT(1);
Y = FR(2) + FL(2) + FC(2) + FNT(2) + YGrav + FT(2); 
Z = FR(3) + FL(3) + FC(3) + FNT(3) + ZGrav + FT(3); 

% Moment Equations
L = ML(1) + MC(1) + MR(1) + MNT(1) + MT(1);
M = ML(2) + MC(2) + MR(2) + MNT(2) + MT(2);
N = ML(3) + MC(3) + MR(3) + MNT(3) + MT(3);

%% Aircraft State Vector [x; y; z; Beta0; Beta1; Beta2; Beta3; u; v; w; p; q; r]
% fofy is the derivative of the state vector, Beta is the Quaternion

fofy(1:3,1) = C'*[u;v;w];

fofy(4,1) = -0.5*(B1*p + B2*q + B3*r);

fofy(5:7,1) = 0.5*[B0 B3 -B2; -B3 B0 B1; B2 -B1 B0]*[p;q;r];
                 
fofy(8:10,1) = [X/m; Y/m; Z/m] - [0 -r q; r 0 -p; -q p 0]*[u;v;w];

fofy(11:13,1) = Iinv*([L;M;N] - [0 -r q; r 0 -p; -q p 0]*I*[p;q;r]);

end

