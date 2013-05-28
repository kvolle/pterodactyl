% Function: [Qinv]=quat_inv(Q)
%
% Description: This function calculates the inverse of a 
%              given Quaternion. 
%
% Input: [Q] is a Quaternion (a 4x1 column vector)
%
% Output:[Qinv] is the Quaternion corresponding to the inverse
%               rotation.
%
% Last Modified: 
%     07/31/97 - Andres Mur-Dongil
%     02/15/98 - G. Chamitoff (Comments, Reduced Computation Time)
%     05/01/98 - G. Chamitoff (Fast Version w/ Only Essential Checks)

function [Qinv]=quat_inv(Q)

% Checking for normality of the quaternion
%Q=q_check(Q);

% Calculating the inverse
Qinv=[Q(1) -Q(2) -Q(3) -Q(4)]';
  
return;  