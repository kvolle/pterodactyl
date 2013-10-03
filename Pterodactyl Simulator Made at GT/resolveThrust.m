function [thrustForce, thrustMoment] = resolveThrust(forces,thrustHTM)
% Takes three arguments of thrust, one for each propeller and converts them
%  to a thrust vector at the center of mass.
% Ignores the rotational momentum of the props

thrustForce = [0;0;0];
thrustMoment = [0;0;0];

for i = 1:3
    tmp = thrustHTM(1:3,1,i)*forces(i);
    thrustForce = thrustForce + tmp;
    thrustMoment = thrustMoment + [0 -thrustHTM(3,4,i), thrustHTM(2,4,i);thrustHTM(3,4,i) 0 -thrustHTM(1,4,i);...
                    -thrustHTM(2,4,i) thrustHTM(1,4,i) 0]*tmp;
end