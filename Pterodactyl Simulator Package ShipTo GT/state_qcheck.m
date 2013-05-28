function [statevec] = state_qcheck(Vec_in)
% Function that simplifies the quaternion check  for the
%   quaternion in the state vector.
%
%     Aircraft State Vector [x; y; z; Beta0; Beta1; Beta2; Beta3; u; v; w; p; q; r]
%     The quaternions are in the elements 4:7 of Vec_in

% Change Log
%{
    9/2012 - Function Written by Trevor Bennett
    5/15/2013 - Code Packaged by Trevor Bennett
%}

statevec(1:3,1) = Vec_in(1:3,1);
statevec(8:13,1) = Vec_in(8:13,1);

% Quaternion Check
Qin = Vec_in(4:7,1);
Q = q_check(Qin);
statevec(4:7,1) = Q;

end