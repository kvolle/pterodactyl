function [htm, area,thrustHTM] = PterodactylGeometry

span = 10;
croot = 3;
ctip  = 2;

% Center of mass
comX = -1.2667;
comY = 0;
comZ = 0;

% Hinge Geometry
hingeY = 1.75;  % Distance from center to hinge/LE intersection
psi = 15*pi/180;
phi    = 00*pi/180;     % Angle of the hinge, tips down for positive angle

%Propeller Position
hingeToProp = 0.5;

% Body Y position of hinge intersection with trailing edge
intY = (croot - hingeY/tan(-psi))/(2*(croot-ctip)/span - 1/tan(-psi));
intX = (intY-hingeY)/tan(-psi);

% Section definiton
% 1-22 is right half, 23-24 is left half
centSectionWidth = hingeY/10;
tmp = centSectionWidth/2;

%Memory allocation
htm = zeros(4,4,44);
area =zeros(1,44);

%Center section
for i = 1:10
    htm(1:4,1:4,i) = [1 0 0 -comX;0 1 0 -comY; 0 0 1 -comZ;0 0 0 1]*...
                     [1 0 0 ((i-0.5)*centSectionWidth)*(croot-ctip)/(2*span)-croot/4;...
                      0 1 0 (i-0.5)*centSectionWidth;...
                      0 0 1 0; 0 0 0 1];
    area(i) = -centSectionWidth*((i-0.5)*centSectionWidth*2*(croot-ctip)/span -croot);
end
for i = 1:10
    htm(1:4,1:4,22+i) = [1 0 0 -comX;0 1 0 -comY; 0 0 1 -comZ;0 0 0 1]*...
                        [1 0 0 ((i-0.5)*centSectionWidth)*(croot-ctip)/(2*span)-croot/4;...
                         0 1 0 -(i-0.5)*centSectionWidth;...
                         0 0 1 0; 0 0 0 1];
    area(22+i) = -centSectionWidth*((i-0.5)*centSectionWidth*2*(croot-ctip)/span -croot);
end
% End Sections
tipSectionWidth = (span/2-intY)/10;

for i=1:10
    htm(1:4,1:4,12+i)= [1 0 0 -comX;0 1 0 -comY; 0 0 1 -comZ;0 0 0 1]*...
          [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 cos(phi) -sin(phi) 0;0 sin(phi) cos(phi) 0;0 0 0 1]*...
          [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 1 0 intY-hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 (intY+(i-0.5)*tipSectionWidth)*(croot-ctip)/(2*span)-croot/4;0 1 0 (i-0.5)*tipSectionWidth;0 0 01 0;0 0 0 1];
    area(12+i) = -tipSectionWidth*((intY+(i-0.5)*tipSectionWidth)*2*(croot-ctip)/span -croot);
end
for i=1:10
    htm(1:4,1:4,34+i)= [1 0 0 -comX;0 1 0 -comY; 0 0 1 -comZ;0 0 0 1]*...
          [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 -hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 cos(-phi) -sin(-phi) 0;0 sin(-phi) cos(-phi) 0;0 0 0 1]*...
          [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 1 0 -intY+hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 (intY+(i-0.5)*tipSectionWidth)*(croot-ctip)/(2*span)-croot/4;0 1 0 -(i-0.5)*tipSectionWidth;0 0 01 0;0 0 0 1];
    area(34+i) = -tipSectionWidth*((intY+(i-0.5)*tipSectionWidth)*2*(croot-ctip)/span -croot);
end

% HTMs of thrust
thrustHTM = zeros(4,4,3);
thrustHTM(:,:,1) = [1 0 0 -comX;0 1 0 -comY; 0 0 1 -comZ;0 0 0 1]*...
          [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 -hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 cos(phi) -sin(phi) 0;0 sin(phi) cos(phi) 0;0 0 0 1]*...
          [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0;0 0 1 0;0 0 0 1]*...
          [1 0 0 0; 0 1 0 -hingeToProp; 0 0 1 0; 0 0 0 1];
thrustHTM(:,:,2) = [1 0 0 -comX;0 1 0 -comY; 0 0 1 -comZ;0  0 0 1];
thrustHTM(:,:,3) = [1 0 0 -comX;0 1 0 -comY; 0 0 1 -comZ;0 0 0 1]*...
          [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 cos(phi) -sin(phi) 0;0 sin(phi) cos(phi) 0;0 0 0 1]*...
          [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 1 0 hingeToProp;0 0 1 0; 0 0 0 1];
      
%MACs of outer triangles
htm(1:4,1:4,12) = [1 0 0 -comX;0 1 0 -comY; 0 0 1 -comZ;0 0 0 1]*...
          [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 cos(phi) -sin(phi) 0;0 sin(phi) cos(phi) 0;0 0 0 1]*...
          [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0;0 0 1 0;0 0 0 1]*...
          [1 0 0 intX/3;0 1 0 2*(intY-hingeY)/3;0 0 1 0; 0 0 0 1];
htm(1:4,1:4,34) = [1 0 0 -comX;0 1 0 -comY; 0 0 1 -comZ;0 0 0 1]*...
          [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 -hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 cos(-phi) -sin(-phi) 0;0 sin(-phi) cos(-phi) 0;0 0 0 1]*...
          [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0;0 0 1 0;0 0 0 1]*...
          [1 0 0 intX/3;0 1 0 -2*(intY-hingeY)/3;0 0 1 0; 0 0 0 1];
area(12) = -0.5*(intY-hingeY)*intX;
area(34) = -0.5*(intY-hingeY)*intX;

% MACs of inner triangles
% X Coord is arbitrary, get advice from Dr Rogers
htm(1:4,1:4,11) = [1 0 0 -comX;0 1 0 -comY; 0 0 1 -comZ;0 0 0 1]*...
                  [1 0 0 (intX+2*(croot-ctip)*(hingeY+(intY-hingeY)/3)/span - croot)/3;0 1 0 hingeY+(intY-hingeY)/3;0 0 1 0; 0 0 0 1];
htm(1:4,1:4,33) = [1 0 0 -comX;0 1 0 -comY; 0 0 1 -comZ;0 0 0 1]*...
                  [1 0 0 (intX+2*(croot-ctip)*(hingeY+(intY-hingeY)/3)/span - croot)/3;0 1 0 -hingeY-(intY-hingeY)/3;0 0 1 0; 0 0 0 1];
area(11) = -0.5*(intY-hingeY)*(2*(croot-ctip)*hingeY/span-croot);
area(33) = -0.5*(intY-hingeY)*(2*(croot-ctip)*hingeY/span-croot);
%{
for i = 1:10
plot(htm(2,4,i),htm(1,4,i),'g*');
plot(htm(2,4,i+22),htm(1,4,i+22),'rh');
hold on
end
%}
for i= 1:44
    plot(i,htm(3,4,i),'gh');
    hold on
end
end
