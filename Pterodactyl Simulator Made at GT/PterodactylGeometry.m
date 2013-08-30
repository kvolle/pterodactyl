function PterodactylGeometry

span = 10;
croot = 3;
ctip  = 2;

% Plot the outer edge of the planform based on the above definitions
leadingEdgeY = linspace(-span/2,span/2,1000);
tipsX = linspace(0,-ctip,400);
trailingY = linspace(-span/2,0,500);
trailingX = linspace(-ctip,-croot,500);

axis([-(span/2+1) (span/2+1) -(croot+1) 1]);
hold on

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

% Plot the quarter chord line for fully extended wings
%  This will need to be amended later
qcX = linspace(-ctip/4,-croot/4,15);
qcY = linspace(-span/2,0,15);
% plot left quarter chord line
plot(qcY,qcX,'b*');
% plot right quarter chord line
plot(qcY+span/2,fliplr(qcX),'b*');
