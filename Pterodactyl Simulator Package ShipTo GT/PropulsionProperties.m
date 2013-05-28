function [LTPoiVec,CTPoiVec,RTPoiVec] = PropulsionProperties(LWCoords,CWCoords,RWCoords)
% Function File to Designate Propulsion Properties for the three propellor
% driven motors mounted on the Pterodactyl

% Change Log
%{
    11/2012 - Function Written by Trevor Bennett
    12/2012 - Propwash Effects Added by Trevor Bennett
    5/15/2013 - Code Packaged by Trevor Bennett
%}

%% Motor Characteristics %%
global MaxMAR

% Max Motor Angular Rate (Function of electronics and battery also)
maxRPM = 12000;
MaxMAR = maxRPM*pi/30;

%% Thrust Lines %%
global ProjR RotR ProjL RotL
global RHingeCoord LHingeCoord CG
global LPropEffect CPropEffect RPropEffect 

% Unfolded Thrust Line Locations and Vectors
% Right Wing Thrust
RsHub = [0.25, 4, 0];   % Right Disk Hub Location [ft]
rsTL = [1 0 0];         % Thrust Line Emminating from Right Hub
RsTL = rsTL/norm(rsTL); % Unit Vector Thrust Line

% Left Wing Thrust
LsHub = [0.25, -4, 0];  % Left Disk Hub Location [ft]
lsTL = [1 0 0];         % Thrust Line Emminating from Left Hub
LsTL = lsTL/norm(lsTL); % Unit Vector Thrust Line

% Center Wing Thrust (Does Not Change)
CHub = [0.25, 0, 0];    % Center Disk Hub Location [ft]
cTL = [1 0 0];          % Thrust Line Emminating from Center Hub
CTL = cTL/norm(cTL);    % Unit Vector Thrust Line
CTPoiVec = [CHub - CG;CTL];


%% Folded Locations and Thrust Lines %%
global HubCoords ThrustLines TMap TMapInv TLMapInv

% Right Folded Thrust Vector
Rtemp = [RsHub;RsTL] - [RHingeCoord;0 0 0];
RtempRot = ProjR*Rtemp';
RTPoiVec = (RotR'*RtempRot)' + [RHingeCoord;0 0 0] - [CG;0 0 0];

% Right Folded Thrust Vector
Ltemp = [LsHub;LsTL] - [LHingeCoord;0 0 0];
LtempRot = ProjL*Ltemp';
LTPoiVec = (RotL'*LtempRot)' + [LHingeCoord;0 0 0] - [CG;0 0 0];

% Hub Coordinates
HubCoords = [LTPoiVec(1,:); CTPoiVec(1,:); RTPoiVec(1,:)];
ThrustLines = [LTPoiVec(2,:); CTPoiVec(2,:); RTPoiVec(2,:)];
TLMapInv = (ThrustLines^-1)';

% Thrust Mapping Matrix From Thrust to Moments
MTL = [(HubCoords(1,2)*ThrustLines(1,3)-HubCoords(1,3)*ThrustLines(1,2));
       (HubCoords(1,3)*ThrustLines(1,1)-HubCoords(1,1)*ThrustLines(1,3));
       (HubCoords(1,1)*ThrustLines(1,2)-HubCoords(1,2)*ThrustLines(1,1))];
   
MTC = [(HubCoords(2,2)*ThrustLines(2,3)-HubCoords(2,3)*ThrustLines(2,2));
       (HubCoords(2,3)*ThrustLines(2,1)-HubCoords(2,1)*ThrustLines(2,3));
       (HubCoords(2,1)*ThrustLines(2,2)-HubCoords(2,2)*ThrustLines(2,1))];

MTR = [(HubCoords(3,2)*ThrustLines(3,3)-HubCoords(3,3)*ThrustLines(3,2));
       (HubCoords(3,3)*ThrustLines(3,1)-HubCoords(3,1)*ThrustLines(3,3));
       (HubCoords(3,1)*ThrustLines(3,2)-HubCoords(3,2)*ThrustLines(3,1))];
   
TMap = [MTL MTC MTR];
TMapInv = TMap^-1;

%% Propeller Coefficient Data %%
global BladeN BladeR BladeType Bladex BladeCLAlpha BladeCD0 BladeCL0
global BladePitch BladeChord BladeDir

BladeN = 2;                 % Number of Blades per Propeller
BladeDir = [1,1,1];         % +1 for Positive Rotation around Body X axis in Straight Wing Configuration

% Propeller Specifications Format: Diam x Pitch
BladeDiam = 20;             % Diameter [in]
PitchSpec1 = 8;             % Pitch Specified by Propeller

BladeR = BladeDiam/24;      % Blade Radius [ft]

BladeType = [1,2];          % Ability to have different propellers on same aircraft

% Computational Locations along Radius of Blade (r/R)
Bladex = [0.001,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0];

% Lift Coefficient of Blade Cross Sections at Bladex Locations [rad]
BladeCL0 = 0.3;
BladeCLAlpha = ...
    [6
     6
     6
     6
     6
     6
     6
     6
     6
     6
     6];
 
% Drag Coefficient of Blade Cross Sections at Bladex Locations [rad]
BladeCD0 = ...
    [0.03
     0.03
     0.03
     0.03
     0.03
     0.03
     0.03
     0.03
     0.03
     0.03
     0.03];

% Pitch of Blade at the Bladex Locations [rad]
% Blade Pitch for Diameter and Pitch

BladePitch = zeros(11,1);
for i = 1:11
    BladePitch(i,1) = atan((PitchSpec1/BladeDiam)/(pi*Bladex(i)));
end

% Chord of the Blade at the Bladex Locations [ft]
BladeChord = ...
   [0.0833
    0.0833
    0.0833
    0.0833
    0.0833
    0.0833
    0.0833
    0.0833
    0.0833
    0.0833
    0.0833];

%% Propeller Effect Vector %%
% This vector detremines whether the induced velocity effects the specific
% strip wihin PterodactylKinetics. Possible values: 0 or 1.

ln = length(LWCoords);
cn = length(CWCoords);
rn = length(RWCoords);

% Allcoate for PropEffects
LPropEffect = zeros(ln,2);
CPropEffect = zeros(cn,2);
RPropEffect = zeros(rn,2);

% PropEffects Regions
ly1 = LsHub(2) - BladeR/2;
ly2 = LsHub(2) + BladeR/2;
cy1 = CHub(2) - BladeR/2;
cy2 = CHub(2) + BladeR/2;
ry1 = RsHub(2) - BladeR/2;
ry2 = RsHub(2) + BladeR/2;

for i=1:ln
    y = LWCoords(i,2);
    if ((y>=ly1) && (y<=ly2)) 
        LPropEffect(i,1) = 1;
        LPropEffect(i,2) = abs(y-LsHub(2))*2/BladeR;
    end
end

for i=1:cn
    y = CWCoords(i,2);
    if ((y>=cy1) && (y<=cy2)) 
        CPropEffect(i,1) = 1;
        CPropEffect(i,2) = abs(y-CHub(2))*2/BladeR;
    end
end

for i=1:rn
    y = RWCoords(i,2);
    if ((y>=ry1) && (y<=ry2)) 
        RPropEffect(i,1) = 1;
        RPropEffect(i,2) = abs(y-RsHub(2))*2/BladeR;
    end
end

end