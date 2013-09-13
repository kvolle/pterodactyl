function PterodactylGeometry
% Function File to Designate Aircraft Properties for the Pterodactyl
% All dimension are measured in the positive sense of the body fixed
% coordinate frame.

% Change Log
%{
    9/2012 - Function Written by Trevor Bennett
    5/2013 - Control Surfaces Added by Trevor Bennett
    5/15/2013 - Code Packaged by Trevor Bennett
%}

%% Aircraft Geometry %%
%%% Mass Properties %%%
global g m CG I Iinv NegArea b wn
g = 32.2;                           % [ft/s/s]
W = 46.18;                          % [lbsf]
m = W/g;                            % [lbsm]

CG = [-1.20,0.03,0.81];             % CG Relative to Center of LE in Body Coords [ft]

I = [170.88, -1.41, -6.03;          % Moment of Inertia Matrix
     -1.41, 73.82, 0.15; 
     -6.03, 0.15, 140.64];
 
Iinv = I^-1;

% Natural Frequency (For Gust Model)
wn = 2;                     % [rad/s]

%%% Dimensions %%%
b = 10;                     % Span [ft]
croot = 3;                  % Root Chord [ft]
ctip = 2;                   % Tip Chord [ft]
S = b*(croot + ctip)/2;     % Wing Area [ft^2]

csweep = -2*(croot-ctip)/b; % Slope of 1/4 Chord Sweep Forward

%%% Control Surfaces %%%
% Right Aileron
rAil_hinge = 19.5/12;       % Measured from leading edge to hingeline [in -> ft]
rAil_in = 35/12;            % Right Aileron Inboard Station [in -> ft]
rAil_out = 60/12;           % Right Aileron Outboard Station [in -> ft]
% Left Aileron
lAil_hinge = 19.5/12;       % Measured from leading edge to hingeline [in -> ft]
lAil_in = -35/12;           % Left Aileron Inboard Station [in -> ft]
lAil_out = -60/12;          % Left Aileron Outboard Station [in -> ft]
% Center Elevator
elev_hinge = 24/12;       % Measured from leading edge to hingeline [in -> ft]
elev_ls = -14/12;           % Elevator Left Station [in -> ft]
elev_rs = 14/12;            % Elevator Right Station [in -> ft]

%%% Fold Geometry %%%
y_rfold = 1.75;             % Right Wing Y Coordinate for Hinge Line at LE
rfold_ang = 20*pi/180;      % Right Wing Fold Line Angle From Body X Axis [Deg -> Rad]
rhinge_ang = 90*pi/180;     % Right Wing Hing Angle From Body Y Axis [Deg -> Rad]

y_lfold = -1.75;            % Left Wing Y Coordinate for Hinge Line at LE
lfold_ang = 22*pi/180;      % Left Wing Fold Line Angle From Body X Axis [Deg -> Rad]
lhinge_ang = 90*pi/180;     % Left Wing Hinge Angle From Body Y Axis [Deg -> Rad]

% Fold 1/4 Cord Location, Fold TE location
y_rfold_qc = (y_rfold + 0.25*tan(rfold_ang)*croot)/(1 - 0.25*csweep*tan(rfold_ang));
y_lfold_qc = (y_lfold - 0.25*tan(lfold_ang)*croot)/(1 - 0.25*csweep*tan(lfold_ang));

y_rfTE = (y_rfold + tan(rfold_ang)*croot)/(1 - csweep*tan(rfold_ang));
y_lfTE = (y_lfold - tan(lfold_ang)*croot)/(1 - csweep*tan(lfold_ang));

% Neglected Area / Total Wing Area Ratio [0,1]
c_rfLE = (croot + csweep*abs(y_rfold));      % Right Inboard of Fold
c_rfTE = (croot + csweep*abs(y_rfTE));       % Right Outboard of Fold
rnega = abs(y_rfTE - y_rfold)*(c_rfLE + c_rfTE)/2;
c_lfLE = (croot + csweep*abs(y_lfold));      % Left Inboard of Fold
c_lfTE = (croot + csweep*abs(y_lfTE));       % Left Outboard of Fold
lnega = abs(y_lfTE - y_lfold)*(c_lfLE + c_lfTE)/2;

% Check Fold lines and Control Surface Placement
if (y_rfTE>=rAil_in)
    fprintf('\nError: The input geometry is such that the Right Fold Line and Aileron Interfere\n');
end
if (y_lfTE<=lAil_in)
    fprintf('\nError: The input geometry is such that the Left Fold Line and Aileron Interfere\n');
end

NegArea = (rnega + lnega)/S;

%% Triangular Flat Plates for Neglected Region
global NegTriCent NegTriArea NegTriNormal

NegTriSCent = zeros(4,3);       % Straight Wing
NegTriCent = zeros(4,3);        % Folded Configuration
NegTriArea = zeros(4,1);
NegTriNormal = zeros(4,3);

% Left Outboard Triangle
NegTriSCent(1,1:3) = [(-c_lfTE)/3, (y_lfold + 2*y_lfTE)/3, 0];
NegTriArea(1,1) = 0.5*abs(-c_lfTE*(y_lfold - y_lfTE));
% Normal Computed in Folded Wing Function Section

% Left Inboard Triangle
NegTriSCent(2,1:3) = [(-c_lfTE - c_lfLE)/3, (2*y_lfold + y_lfTE)/3, 0];
NegTriCent(2,1:3) = NegTriSCent(2,1:3) - CG;
NegTriArea(2,1) = 0.5*abs(-c_lfLE*(y_lfold - y_lfTE));
NegTriNormal(2,1:3) = [0,0,1];

% Right Inboard Triangle
NegTriSCent(3,1:3) = [(-c_rfTE - c_rfLE)/3, (2*y_rfold + y_rfTE)/3, 0];
NegTriCent(3,1:3) = NegTriSCent(3,1:3) - CG;
NegTriArea(3,1) = 0.5*abs(-c_rfLE*(y_rfold - y_rfTE));
NegTriNormal(3,1:3) = [0,0,1];

% Right Outboard Triangle
NegTriSCent(4,1:3) = [(-c_rfTE)/3, (y_rfold + 2*y_rfTE)/3, 0];
NegTriArea(4,1) = 0.5*abs(-c_rfTE*(y_rfold - y_rfTE));
% Normal Computed in Folded Wing Function Section

% Note: The Normal vector is represented in the body axis

%% Strip Coordinates In Unfolded Configuration %%
% The strips are evenly spaced along the span with one on the centerline
% The coordinates are measured from the leading edge at the centerline
% All strips have the 1/4 chord at the same z coordinate
% Coordinate Development: On Wing -> Folded -> Rel. to CG
% Coordinates stored as [x,y,z]
%       x,y,z are body frame locations

global CStripCoords RStripCoords LStripCoords
global cnstrip rnstrip lnstrip
global cchords rchords lchords
global CStripArea RStripArea LStripArea

% Center Wing Coordinates
% Number of Strips on Center Wing (with Centerline - Must be Odd #)
cnstrip = 21;                                 
cspacer = (y_rfold - y_lfold)/(cnstrip-1);  % Seperation Between Strips Along Y Axis
ZLoc = 0;                                   % Z of the Centerline 1/4 Chord Point Rel. LE 
% (No Dihedral) Therefore all coordinates at this z location

% X,Y,Z Coordinates of Center Wing Strip's 1/4 Chord [ft,ft,ft] from LE
CWingCoords = zeros(cnstrip,3);
CStripCoords = zeros(cnstrip,3);
cchords = zeros(cnstrip,1);
CStripArea = zeros(cnstrip,1);

% Centerline Strip
cline = (cnstrip-1)/2 + 1;
CWingCoords(cline,1) = -0.25*croot;
CWingCoords(cline,2) = 0;
CWingCoords(cline,3) = ZLoc;
CStripCoords(cline,:) = CWingCoords(cline,:) - CG;
cchords(cline,1) = croot;
rc1 = croot + csweep*(cspacer/2);
lc1 = rc1;
CStripArea(cline,1) = cspacer*(croot + rc1)/2;

for i=1:(cnstrip-1)/2
    
    y = i*cspacer;
    chord = (croot + csweep*y);
    rc2 = (croot + csweep*(y+cspacer));
    lc2 = (croot + csweep*(y+cspacer));
    qcy = -0.25*chord;
    
    % Out to Right
    rind = cline+i;
    CWingCoords(rind,1) = qcy;
    CWingCoords(rind,2) = y;
    CWingCoords(rind,3) = ZLoc;
    CStripCoords(rind,:) = CWingCoords(rind,:) - CG;
    cchords(rind,1) = chord;
    CStripArea(rind,1) = cspacer*(rc1 + rc2)/2;
    
    % Out to Left
    lind = cline-i;
    CWingCoords(lind,1) = qcy;
    CWingCoords(lind,2) = -y;
    CWingCoords(lind,3) = ZLoc;  
    CStripCoords(lind,:) = CWingCoords(lind,:) - CG;
    cchords(lind,1) = chord;
    CStripArea(lind,1) = cspacer*(lc1 + lc2)/2;
    
    rc1 = rc2;
    lc1 = lc2;
end
% Correct the Half Strips by Hinge
CStripArea(cnstrip,1) = 0.5*cspacer*(2*c_rfLE - csweep*(cspacer/2))/2;
CStripArea(1,1) = 0.5*cspacer*(2*c_lfLE - csweep*(cspacer/2))/2;

% Right Wing Coordinates
% Number of Strips on Right Wing (Includes Fold Strip and Wing Tip Strip)
rnstrip = 10;                               
rspacer = (b/2 - abs(y_rfTE))/(rnstrip-1);  % Seperation Between Strips Along Y Axis
ZLoc = 0;                                   % Z of the Centerline 1/4 Chord Point Rel. LE 
% (No Dihedral) Therefore all coordinates at this z location

% X,Y,Z Coordinates of Center Wing Strip's 1/4 Chord [ft,ft,ft] from LE
RWingCoords = zeros(rnstrip,3);
rchords = zeros(rnstrip,1);
RStripArea = zeros(rnstrip,1);

% Fold Line Strip
RWingCoords(1,1) = -0.25*(croot + csweep*y_rfTE);
RWingCoords(1,2) = y_rfTE;
RWingCoords(1,3) = ZLoc;
rchords(1,1) = (croot + csweep*y_rfTE);
rc1 = c_rfTE + csweep*(rspacer/2);
RStripArea(1,1) = 0.5*rspacer*(c_rfTE + rc1)/2;

for i=1:rnstrip-1
    
    y = y_rfTE +i*rspacer;
    chord = (croot + csweep*abs(y));
    rc2 = (c_rfTE + csweep*(y+rspacer));
    qcy = -0.25*chord;
    
    ind = 1+i;
    RWingCoords(ind,1) = qcy;
    RWingCoords(ind,2) = y;
    RWingCoords(ind,3) = ZLoc;
    rchords(ind,1) = chord;
    RStripArea(ind,1) = rspacer*(rc1 + rc2)/2;
    
    rc1 = rc2;
end
% Correct the Half Strip at Wing Tip
RStripArea(rnstrip,1) = 0.5*rspacer*(2*ctip - csweep*(rspacer/2))/2;

% Left Wing Coordinates
% Number of Strips on Left Wing (Includes Fold Strip and Wing Tip Strip)
lnstrip = 10;                               
lspacer = (b/2 - abs(y_lfTE))/(lnstrip-1);  % Seperation Between Strips Along Y Axis
ZLoc = 0;                                   % Z of the Centerline 1/4 Chord Point Rel. LE 
% (No Dihedral) Therefore all coordinates at this z location

% X,Y,Z Coordinates of Center Wing Strip's 1/4 Chord [ft,ft,ft] from LE
LWingCoords = zeros(lnstrip,3);
lchords = zeros(lnstrip,1);
LStripArea = zeros(lnstrip,1);

% Fold Line Strip
LWingCoords(1,1) = -0.25*(croot + csweep*y_rfTE);
LWingCoords(1,2) = y_lfTE;
LWingCoords(1,3) = ZLoc;
lchords(1,1) = (croot + csweep*y_rfTE);
lc1 = c_lfTE + csweep*(lspacer/2);
LStripArea(1,1) = 0.5*lspacer*(c_lfTE + lc1)/2;

for i=1:rnstrip-1
    
    y = y_lfTE - i*lspacer;
    chord = (croot + csweep*abs(y));
    lc2 = (c_lfTE + csweep*(y+lspacer));
    qcy = -0.25*chord;
    
    ind = 1+i;
    LWingCoords(ind,1) = qcy;
    LWingCoords(ind,2) = y;
    LWingCoords(ind,3) = ZLoc;
    lchords(ind,1) = chord;
    LStripArea(ind,1) = lspacer*(lc1 + lc2)/2;
    
    lc1 = lc2;
end
% Correct the Half Strip at Wing Tip
LStripArea(lnstrip,1) = 0.5*lspacer*(2*ctip - csweep*(lspacer/2))/2;

% Flip Left Wing index direction to left to right
LWingCoords = flipud(LWingCoords);
lchords = flipud(lchords);
LStripArea = flipud(LStripArea);

%% Strip Coordinates In Folded Configuration %%
% Global Variables for Folded Configuration
global RHingeCoord LHingeCoord
global ProjR RotR ProjL RotL

% Left Hinge Point
LHingeCoord(1,1) = -0.25*(croot + csweep*abs(y_lfold_qc));
LHingeCoord(1,2) = y_lfold_qc;
LHingeCoord(1,3) = 0.5*(0.12*(croot + csweep*abs(y_lfold_qc)));
% Right Hinge Point
RHingeCoord(1,1) = -0.25*(croot + csweep*abs(y_rfold_qc));
RHingeCoord(1,2) = y_rfold_qc;
RHingeCoord(1,3) = 0.5*(0.12*(croot + csweep*abs(y_rfold_qc)));

% Left Wing Rotation Matrix (Straight -> Fold)
Ch = cos(lhinge_ang);
Sh = sin(lhinge_ang);
Cf = cos(lfold_ang);
Sf = sin(lfold_ang);
% Positive Rotation about Body Z Axis
ProjL =  [Cf Sf 0;-Sf Cf 0;0 0 1];
% Left Rotation Matrix
RotL = [Cf Sf 0;-Ch*Sf Ch*Cf -Sh;-Sh*Sf Sh*Cf Ch];

% Left Wingtip Vector
LTip = [-ctip -b/2 0;0 -b/2 0] - [LHingeCoord;LHingeCoord];
LTipRot = ProjL*LTip';
LWingTip = (RotL'*LTipRot)' + [LHingeCoord;LHingeCoord] - [CG;CG];

% Left Hinge Rotation
LFoldCoords = zeros(lnstrip,3);
LStripCoords = zeros(lnstrip,3);
for i = 1:lnstrip 
    StripCoord = LWingCoords(i,:) - LHingeCoord;
    % Rotated Coordinates into e Frame (Fold Line Frame)
    LFoldCoords(i,:) = (RotL'*(ProjL*StripCoord'))';
    LStripCoords(i,:) = LFoldCoords(i,:) - CG + LHingeCoord;
end

% Right Wing Rotation Matrix (Straight -> Fold)
Ch = cos(rhinge_ang);
Sh = sin(rhinge_ang);
Cf = cos(rfold_ang);
Sf = sin(rfold_ang);
% Negative Rotation about Body Z Axis
ProjR =  [Cf -Sf 0;Sf Cf 0;0 0 1];
% Right Rotation Matrix
RotR = [Cf -Sf 0;Ch*Sf Ch*Cf Sh;-Sh*Sf -Sh*Cf Ch];

% Right Wingtip Vector
RTip = [-ctip b/2 0;0 b/2 0] - [RHingeCoord;RHingeCoord];
RTipRot = ProjR*RTip';
RWingTip = (RotR'*RTipRot)' + [RHingeCoord;RHingeCoord] - [CG;CG];

% Right Hinge Rotation
RFoldCoords = zeros(rnstrip,3);
RStripCoords = zeros(rnstrip,3);
for i = 1:rnstrip 
    StripCoord = RWingCoords(i,:) - RHingeCoord;
    % Rotated Coordinates into e Frame (Fold Line Frame)
    RFoldCoords(i,:) = (RotR'*(ProjR*StripCoord'))';
    RStripCoords(i,:) = RFoldCoords(i,:) - CG + RHingeCoord;
end

% Neglected Triangle Centers and Normals
NegTriCent(1,1:3) = (RotL'*(ProjL*(NegTriSCent(1,1:3) - LHingeCoord)'))' + LHingeCoord - CG;
NegTriCent(4,1:3) = (RotR'*(ProjR*(NegTriSCent(4,1:3) - RHingeCoord)'))' + RHingeCoord - CG;
NegTriNormal(1,1:3) = (RotL'*[0;0;1])';
NegTriNormal(4,1:3) = (RotR'*[0;0;1])';

%% Control Surfaces %%
global ControlSurface
% Populate AileronEffect
ControlSurface.L.L = zeros(lnstrip,2);
ControlSurface.L.lhinge = lAil_hinge;
ailspan = abs(lAil_out - lAil_in);
ControlSurface.L.Sc = ailspan*((croot + csweep*abs(lAil_out)) + (croot + csweep*abs(lAil_in)))/2;
ControlSurface.L.avc = ControlSurface.L.Sc/ailspan;
ControlSurface.L.xbar = ControlSurface.L.avc - ControlSurface.L.lhinge;
y_avc = (ControlSurface.L.avc - croot)/csweep;
ControlSurface.L.r = (RotL'*(ProjL*[-0.25*ControlSurface.L.avc;-y_avc;0]))' - CG + LHingeCoord;
for i=1:lnstrip
    y = LWingCoords(i,2);
    if ((y>=lAil_out) && (y<=lAil_in)) 
        ControlSurface.L.L(i,1) = 1;
        ControlSurface.L.L(i,2) = lchords(i) - lAil_hinge;     % Computation of "xa"
    end
end

ControlSurface.C.C = zeros(cnstrip,2);
ControlSurface.C.chinge = elev_hinge;
elevspan = abs(elev_rs - elev_ls);
ControlSurface.C.Sc = abs(elev_rs)*(2*croot + csweep*abs(elev_rs))/2 + abs(elev_ls)*(2*croot + csweep*abs(elev_ls))/2;
ControlSurface.C.avc = ControlSurface.C.Sc/elevspan;
ControlSurface.C.xbar = ControlSurface.C.avc - ControlSurface.C.chinge;
y_avc = elev_rs - elevspan/2;
ControlSurface.C.r = [-0.25*ControlSurface.C.avc;y_avc;0]' - CG;
for i=1:cnstrip
    y = CWingCoords(i,2);
    if ((y>=elev_ls) && (y<=elev_rs)) 
        ControlSurface.C.C(i,1) = 1;
        ControlSurface.C.C(i,2) = cchords(i) - elev_hinge;     % Computation of "xa"
    end
end

ControlSurface.R.R = zeros(rnstrip,2);
ControlSurface.R.rhinge = rAil_hinge;
ailspan = abs(rAil_out - rAil_in);
ControlSurface.R.Sc = ailspan*((croot + csweep*abs(rAil_out)) + (croot + csweep*abs(rAil_in)))/2;
ControlSurface.R.avc = ControlSurface.R.Sc/ailspan;
ControlSurface.R.xbar = ControlSurface.R.avc - ControlSurface.R.rhinge;
y_avc = (ControlSurface.R.avc - croot)/csweep;
ControlSurface.R.r = (RotR'*(ProjR*[-0.25*ControlSurface.R.avc;y_avc;0]))' - CG + RHingeCoord;
for i=1:rnstrip
    y = RWingCoords(i,2);
    if ((y>=rAil_in) && (y<=rAil_out)) 
        ControlSurface.R.R(i,1) = 1;
        ControlSurface.R.R(i,2) = rchords(i) - rAil_hinge;     % Computation of "xa"
    end
end

%% Aerodynamic and Propulsive Properties %%
global mu Alpha ReCF CFxAlpha CFzAlpha ReCm CmAlpha MeshMult CFxCoeff CFzCoeff CmCoeff
% AerodynamicProperties;            % Run when new aerodynamic properties are inserted.
load('AerodynamicProperties.mat');  % Brings in most recent run of AerodynamicProperties
[LTPoiVec,CTPoiVec,RTPoiVec] = PropulsionProperties(LWingCoords,CWingCoords,RWingCoords);
% Control Linearizations
load('ControlLinearizations.mat');

%% Configuration Plotter %%
global plotNum

Wing = [0,0;0,b/2;-ctip,b/2;-croot,0;-ctip,-b/2;0,-b/2;0,0];
LAil = [-(croot + csweep*abs(lAil_out)),lAil_out;-lAil_hinge,lAil_out;-lAil_hinge,lAil_in;-(croot + csweep*abs(lAil_in)),lAil_in];
RAil = [-(croot + csweep*abs(rAil_out)),rAil_out;-rAil_hinge,rAil_out;-rAil_hinge,rAil_in;-(croot + csweep*abs(rAil_in)),rAil_in];
Elev = [-(croot + csweep*abs(elev_ls)),elev_ls;-elev_hinge,elev_ls;-elev_hinge,elev_rs;-(croot + csweep*abs(elev_rs)),elev_rs];

figure (plotNum)
plotNum = plotNum + 1;
plot(CWingCoords(:,2),CWingCoords(:,1),'*');
hold on
plot(RWingCoords(:,2),RWingCoords(:,1),'*');
plot(LWingCoords(:,2),LWingCoords(:,1),'*');
plot(Wing(:,2),Wing(:,1),'k','LineWidth',2);
plot(LAil(:,2),LAil(:,1),'r','LineWidth',1);
plot(RAil(:,2),RAil(:,1),'r','LineWidth',1);
plot(Elev(:,2),Elev(:,1),'r','LineWidth',1);
plot([y_rfold;y_rfTE],[0;-c_rfTE],'-k','LineWidth',1);
plot([y_lfold;y_lfTE],[0;-c_lfTE],'-k','LineWidth',1);
plot([y_rfold;y_rfold],[0;-c_rfLE],'-.k',[y_rfTE;y_rfTE],[0;-c_rfTE],'-.k','LineWidth',1);
plot([y_lfold;y_lfold],[0;-c_lfLE],'-.k',[y_lfTE;y_lfTE],[0;-c_lfTE],'-.k','LineWidth',1);
plot(NegTriSCent(:,2),NegTriSCent(:,1),'+b')
plot(CG(2),CG(1),'or')
axis([-6 6 -4 1])
titlestring = sprintf('Unfolded Locations of Quater Chord Computation Points');
title(titlestring)
xlabel('Body Y Axis')
ylabel('Body X Axis')
hold off

figure (plotNum)
plotNum = plotNum + 1;
% Computation Points
plot3(CStripCoords(:,1),CStripCoords(:,2),CStripCoords(:,3),'*');
hold on
plot3(RStripCoords(:,1),RStripCoords(:,2),RStripCoords(:,3),'*');
plot3(LStripCoords(:,1),LStripCoords(:,2),LStripCoords(:,3),'*');
plot3(NegTriCent(:,1),NegTriCent(:,2),NegTriCent(:,3),'+b')
% CG Location - Coordinate System Origin
plot3(0,0,0,'or')
% Thrust Lines
quiver3(CTPoiVec(1,1),CTPoiVec(1,2),CTPoiVec(1,3),CTPoiVec(2,1),CTPoiVec(2,2),CTPoiVec(2,3));
quiver3(RTPoiVec(1,1),RTPoiVec(1,2),RTPoiVec(1,3),RTPoiVec(2,1),RTPoiVec(2,2),RTPoiVec(2,3));
quiver3(LTPoiVec(1,1),LTPoiVec(1,2),LTPoiVec(1,3),LTPoiVec(2,1),LTPoiVec(2,2),LTPoiVec(2,3));
% Wing Tips
quiver3(RWingTip(1,1),RWingTip(1,2),RWingTip(1,3),RWingTip(2,1)-RWingTip(1,1),RWingTip(2,2)-RWingTip(1,2),RWingTip(2,3)-RWingTip(1,3));
quiver3(LWingTip(1,1),LWingTip(1,2),LWingTip(1,3),LWingTip(2,1)-LWingTip(1,1),LWingTip(2,2)-LWingTip(1,2),LWingTip(2,3)-LWingTip(1,3));
hold off
set(gca,'zdir','reverse')
set(gca,'ydir','reverse')
axis equal
title('Folded Locations with Thrust Vectors')
xlabel('Body X Axis')
ylabel('Body Y Axis')
zlabel('Body Z Axis')

end
