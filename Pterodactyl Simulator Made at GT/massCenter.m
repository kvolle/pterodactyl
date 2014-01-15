function [comX, comY, comZ,Ixx,Iyy,Izz] = massCenter(m,phiL,phiR,psi,hingeY)
%planform geometry
span = 10;
croot = 3;
ctip  = 2;

% ANGLES IN RADIANS %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
if (psi>pi)
    psi = psi*pi/180;
end
if (phiL>pi)
    phiL = phiL*pi/180;
end
if (phiR>pi)
    phiR = phiR*pi/180;
end
% Hinge Geometry
%hingeY = 1.75;  % Distance from center to hinge/LE intersection


intY = (croot - hingeY/tan(-psi))/(2*(croot-ctip)/span - 1/tan(-psi));
centSectionWidth = hingeY/10;
tipSectionWidth = (span/2-intY)/10;

htm=zeros(4,4,44);
Ix=zeros(1,44);
Iy=zeros(1,44);
Iz=zeros(1,44);
volume = zeros(1,44);
chord = zeros(1,44);
y = zeros(1,44);

%Center section
for i = 1:10
    htm(1:4,1:4,i) = [1 0 0 -0.4*croot+0.4*((i-0.5)*centSectionWidth)*2*(croot-ctip)/span;...
                      0 1 0 (i-0.5)*centSectionWidth;...
                      0 0 1 0; 0 0 0 1];
    y(i) = (i-0.5)*centSectionWidth;
    chord(i) = 3-0.2*y(i);
    volume(i) = centSectionWidth*0.08217*chord(i)^2;
    %volume(i) = 0.082206*centSectionWidth*(croot-((i-0.5)*centSectionWidth)*2*(croot-ctip)/span)^2;
end
for i = 1:10
    htm(1:4,1:4,22+i) = [1 0 0 -0.4*croot+0.4*((i-0.5)*centSectionWidth)*2*(croot-ctip)/span;...
                         0 1 0 -(i-0.5)*centSectionWidth;...
                         0 0 1 0; 0 0 0 1];
    y(22+i) = -(i-0.5)*centSectionWidth;
    chord(22+i) = 3+0.2*y(22+i);
    volume(22+i) = centSectionWidth*0.08217*chord(i)^2;
    %volume(22+i) = 0.082206*centSectionWidth*(croot-((i-0.5)*centSectionWidth)*2*(croot-ctip)/span)^2;
end
    % End Sections

for i=1:10
    htm(1:4,1:4,12+i)= [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 cos(phiR) -sin(phiR) 0;0 sin(phiR) cos(phiR) 0;0 0 0 1]*...
          [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 0;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 1 0 intY-hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0.4*(intY+(i-0.5)*tipSectionWidth)*2*(croot-ctip)/span-0.4*croot;0 1 0 (i-0.5)*tipSectionWidth;0 0 01 0;0 0 0 1];
    y(12+i) = (intY+(i-0.5)*tipSectionWidth);
    chord(12+i) = 3- 0.2*y(12+i);
    volume(12+i) = 0.082206*tipSectionWidth*chord(12+i)^2;
    %volume(12+i) = 0.082206*tipSectionWidth*(croot-(intY+(i-0.5)*tipSectionWidth)*2*(croot-ctip)/span)^2;
end
for i=1:10
    htm(1:4,1:4,34+i)= [cos(psi) -sin(psi) 0 0;sin(psi) cos(psi) 0 -hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 cos(-phiL) -sin(-phiL) 0;0 sin(-phiL) cos(-phiL) 0;0 0 0 1]*...
          [cos(-psi) -sin(-psi) 0 0;sin(-psi) cos(-psi) 0 0;0 0 1 0;0 0 0 1]*...
          [1 0 0 0;0 1 0 -intY+hingeY;0 0 1 0;0 0 0 1]*...
          [1 0 0 0.4*(intY+(i-0.5)*tipSectionWidth)*2*(croot-ctip)/span-0.4*croot;0 1 0 -(i-0.5)*tipSectionWidth;0 0 01 0;0 0 0 1];
    y(34+i) = -(intY+(i-0.5)*tipSectionWidth);
    chord(34+i) = 3 + 0.2*y(34+i);
    volume(34+i) = 0.082206*tipSectionWidth*chord(34+i)^2;
    %volume(34+i) = 0.082206*tipSectionWidth*(croot-(intY+(i-0.5)*tipSectionWidth)*2*(croot-ctip)/span)^2;
    
end

for i = 1:44
    Ix(i) = volume(i)*(m/sum(volume)*(htm(2,4,i)^2+htm(3,4,i)^2)+0.18^2 + 0.30^2);
    Iy(i) = volume(i)*(m/sum(volume)*(htm(1,4,i)^2+htm(3,4,i)^2)+2.50^2 + 0.30^2);
    Iz(i) = volume(i)*(m/sum(volume)*(htm(1,4,i)^2+htm(2,4,i)^2)+0.18^2 + 2.50^2);
end

%{
plot(Ix,'r.');
hold on
plot(Iy,'g.');
plot(Iz,'b.');
%}
Ixx = sum(Ix);
Iyy = sum(Iy);
Izz = sum(Iz);

%disp(Ixx)
%disp(Iyy)
%disp(Izz)
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

%{
hold on

for i = 1:44
    plot3(htm(2,4,i),htm(1,4,i),-htm(3,4,i),'r*');
    pause(0.01)
end
%}
%{
plot(volume);
figure(2)
plot(y,chord,'.')
intY
hingeY
tipSectionWidth
%}
end