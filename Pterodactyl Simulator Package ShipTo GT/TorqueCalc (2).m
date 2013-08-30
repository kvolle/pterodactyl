function [TorqueOut] = TorqueCalc(Omega,vClimb)

p00 = -1.452142e-002;
p10 = 5.300510e-005;
p01 = -1.792692e-003;
p20 = 2.337961e-006;
p11 = 7.546314e-006;
p02 = -6.503713e-005;

TorqueOut = p20*Omega^2 + p11*Omega*vClimb + p10*Omega + p02*vClimb^2 + p01*vClimb + p00;

end