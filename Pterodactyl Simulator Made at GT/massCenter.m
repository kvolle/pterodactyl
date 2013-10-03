function [comX, comY, comZ] = massCenter(phiL,phiR);
%planform geometry
span = 10;
croot = 3;
ctip  = 2;

% Hinge Geometry
hingeY = 1.75;  % Distance from center to hinge/LE intersection
psi = 30*pi/180;


intY = (croot - hingeY/tan(-psi))/(2*(croot-ctip)/span - 1/tan(-psi));
centSectionWidth = hingeY/10;
tipSectionWidth = (span/2-intY)/10;

htm=zeros(4,4,44);
%volume = zeros(1,44);
%Center section
for i = 1:10
    htm(1:4,1:4,i) = [1 0 0 -0.4*croot+0.4*((i-0.5)*centSectionWidth)*2*(croot-ctip)/span;...
                      0 1 0 (i-0.5)*centSectionWidth;...
                      0 0 1 0; 0 0 0 1];
    volume(i) = 0.08217*centSectionWidth*(croot-((i-0.5)*centSectionWidth)*2*(croot-ctip)/span);
end
for i = 1:10
    htm(1:4,1:4,22+i) = [1 0 0 -0.4*croot+0.4*((i-0.5)*centSectionWidth)*2*(croot-ctip)/span;...
                         0 1 0 -(i-0.5)*centSectionWidth;...
                         0 0 1 0; 0 0 0 1];
    volume(22+i) = 0.08217*centSectionWidth*(croot-((i-0.5)*centSectionWidth)*2*(croot-ctip)/span);
end
    % End Sections

for i=1:10
    htm(1:4,1:4,12+i)= [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 cos(phiR) -sin(phiR) 0;0 sin(phiR) cos(phiR) 0;0 0 0 1]*...
          [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 1 0 intY-hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0.4*(intY+(i-0.5)*tipSectionWidth)*2*(croot-ctip)/span-0.4*croot;0 1 0 (i-0.5)*tipSectionWidth;0 0 01 0;0 0 0 1];
    volume(12+i) = 0.08217*tipSectionWidth*(croot-(intY+(i-0.5)*tipSectionWidth)*2*(croot-ctip)/span);
end
for i=1:10
    htm(1:4,1:4,34+i)= [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 -hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 cos(-phiL) -sin(-phiL) 0;0 sin(-phiL) cos(-phiL) 0;0 0 0 1]*...
          [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 1 0 -intY+hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0.4*(intY+(i-0.5)*tipSectionWidth)*2*(croot-ctip)/span-0.4*croot;0 1 0 -(i-0.5)*tipSectionWidth;0 0 01 0;0 0 0 1];
    volume(34+i) = 0.08217*tipSectionWidth*(croot-(intY+(i-0.5)*tipSectionWidth)*2*(croot-ctip)/span);
end


% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Don't have the triangles around the hinges accounted for
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
comX = 0;
comY = 0;
comZ = 0;
sumVol = sum(volume);
for i=1:44
    comX = comX + htm(1,4,i)*volume(i)/sumVol;
    comY = comY + htm(2,4,i)*volume(i)/sumVol;
    comZ = comZ + htm(3,4,i)*volume(i)/sumVol;
end
if (abs(comX)<10^-7)
    comX = 0;
end
if (abs(comY)<10^-7)
    comY = 0;
end
if (abs(comZ)<10^-7)
    comZ = 0;
end
end