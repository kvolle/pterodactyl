function [aeroForces, aeroMoments] = resolveForces (htm, lift,drag,pitchMoment)
aeroForces= [0;0;0];
aeroMoments = [0;0;0];


for i = 1:44
    tmp = -htm(1:3,3,i)*lift(i)- htm(1:3,1,i)*drag(i);
    aeroForces = aeroForces + tmp;
    aeroMoments = aeroMoments + ([0 -htm(3,4,i) htm(2,4,i);htm(3,4,i) 0 -htm(1,4,i);-htm(2,4,i) htm(1,4,i) 0]*tmp) + htm(1:3,2,i)*pitchMoment(i);
end