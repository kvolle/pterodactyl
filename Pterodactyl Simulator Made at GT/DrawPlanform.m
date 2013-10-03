function DrawPlanform
% This function draws the planform of Pterodactyl
tic

span = 10;
croot = 3;
ctip  = 2;

% Center of mass
comX = -1.2667;
comY = 0;
comZ = 0;

leadingEdgeY = linspace(-span/2,span/2,1000);
tipsX = linspace(0,-ctip,400);
trailingY = linspace(-span/2,0,500);
trailingX = linspace(-ctip,-croot,500);

axis([-(span/2+1) (span/2+1) -(croot+1) 1]);
hold on
axis equal

% Axis labels to match AE convention
xlabel('Y axis')
ylabel('X axis');
title('Pterodactyl Planform Geometry');

% plot leading edge
plot(leadingEdgeY,0,'k.-','LineWidth',100);
% plot left tip
plot(-span/2,tipsX,'k.-','LineWidth',100);
% plot right tip
plot(span/2,tipsX,'k.-','LineWidth',100);
% plot left trailing edge;
plot(trailingY,trailingX,'k.-');
%plot right trailing edge
plot(trailingY+span/2,fliplr(trailingX),'k.-');

%plot center of pass
plot(comY,comX,'ro');

% Hinge Geometry
hingeY = 1.75;  % Distance from center to hinge/LE intersection
hingeD = 15;    % Angle from chord at -hingeY to hinge line on left side
psi = hingeD*pi/180;
phi    = 0;     % Angle of the hinge, tips down for positive angle

% Body Y position of hinge intersection with trailing edge
intY = (croot - hingeY/tan(-psi))/(2*(croot-ctip)/span - 1/tan(-psi));
intX = (intY-hingeY)/tan(-psi);

HingeX = linspace(0,intX,100);
rightHingeY = linspace(hingeY,intY,100);
leftHingeY = linspace(-hingeY,-intY,100);

plot(rightHingeY,HingeX,'k-.');
plot(leftHingeY,HingeX,'k-.');

%plot outer edges of triangular sections
plot(intY,linspace(0,intX),'r--');
plot(-intY,linspace(0,intX),'r--');
%plot inner edges of triangular sections
plot(-hingeY,linspace(0,(2*(ctip-croot)/span)*(span/2 - hingeY) -ctip),'r--');
plot(hingeY,linspace(0,(2*(ctip-croot)/span)*(span/2 - hingeY) -ctip),'r--');

[htm,~,~] = PterodactylGeometry;
for i = 1:44
    plot(htm(2,4,i)+comY,htm(1,4,i)+comX,'g*');
end
toc