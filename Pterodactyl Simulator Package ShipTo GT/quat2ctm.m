function [C]=quat2ctm(beta)

% Change Log
%{
    10/2012 - Function Written by Trevor Bennett
%}

b0=beta(1);b1=beta(2);b2=beta(3);b3=beta(4);
C=[b0^2+b1^2-b2^2-b3^2 2*(b1*b2+b0*b3) 2*(b1*b3-b0*b2)
   2*(b1*b2-b0*b3)  b0^2-b1^2+b2^2-b3^2 2*(b2*b3+b0*b1)
   2*(b1*b3+b0*b2) 2*(b2*b3-b0*b1)  b0^2-b1^2-b2^2+b3^2];
end
