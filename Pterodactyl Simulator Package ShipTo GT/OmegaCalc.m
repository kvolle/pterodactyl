function [OmegaCMD] = OmegaCalc(Thrust,vClimb)

p00 = 1.689206e-002;
p10 = -3.346657e-005;
p01 = -1.134379e-003;
p20 = 3.463976e-005;
p11 = -1.452247e-004;
p02 = -7.478217e-004;

OmegaCMD = -(p10 + p11*vClimb + (p10^2 + 2*p10*p11*vClimb + p11^2*vClimb^2 - 4*p02*p20*vClimb^2 - 4*p01*p20*vClimb + 4*Thrust*p20 - 4*p00*p20)^(1/2))/(2*p20);

end