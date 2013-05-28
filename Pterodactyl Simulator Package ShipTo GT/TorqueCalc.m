function [TorqueOut] = TorqueCalc(Omega,vClimb)

p00 = -1.452142e-02;
p10 = 5.300510e-05;
p01 = -1.792692e-03;
p20 = 2.337961e-06;
p11 = 7.546314e-06;
p02 = -6.503713e-05;

TorqueOut = p00 + Omega*p10 + p01*vClimb + Omega^2*p20 + p02*vClimb^2 + Omega*p11*vClimb;

end