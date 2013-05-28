% Function:  [Q]=ctm2quat(ctm)
%
% Description:  This program determines the Quaternion corresponding
%               to a given Coordinate Transformation Matrix.

% Input:  [ctm] is a Coordinate Transformation Matrix.
%
% Output: [Q] is the corresponding Quaternion.
%
% Last Modified:
%     06/20/96 - A. Mur-Dongil
%     02/10/97 - T. Carter
%     07/14/97 - A. Mur-Dongil
%     02/15/98 - G. Chamitoff (New Algorithm Needed to Protect Against 
%                              Division by Zero, and to Handle Singular
%                              Conditions)
%     05/01/98 - G. Chamitoff (Fast Version w/ Only Essential Checks)
%

function[Q]=ctm2quat(ctm)

% Checking for Orthonormality
%C = ctm_chck(ctm);
C = ctm;
    
% Calculating the Quaternion
% Scalar Component 
B0 = 0.5*real(sqrt(1+trace(C)));
    
% Vector Component
B1 = 0.5*real(sqrt(1+2*C(1,1)-trace(C)));
B2 = 0.5*real(sqrt(1+2*C(2,2)-trace(C)));
B3 = 0.5*real(sqrt(1+2*C(3,3)-trace(C)));
    
% Find Largest Bi
[Bmax,Imax] = max([B0 B1 B2 B3]);
    
% Compute Other Bi Via Stanley Method
B0B1 = (C(2,3)-C(3,2))/4;
B0B2 = (C(3,1)-C(1,3))/4;
B0B3 = (C(1,2)-C(2,1))/4;
B2B3 = (C(2,3)+C(3,2))/4;
B3B1 = (C(3,1)+C(1,3))/4;
B1B2 = (C(1,2)+C(2,1))/4;
if (Imax==1)      % Bmax=B0
    B1 = B0B1/B0;
    B2 = B0B2/B0;
    B3 = B0B3/B0;
elseif (Imax==2)  % Bmax=B1
    B0 = B0B1/B1;
    B2 = B1B2/B1;
    B3 = B3B1/B1;
elseif (Imax==3)  % Bmax=B2
    B0 = B0B2/B2;
    B1 = B1B2/B2;
    B3 = B2B3/B2;
else              % Bmax=B3
    B0 = B0B3/B3;
    B1 = B3B1/B3;
    B2 = B2B3/B3;
end;
    
% Quaternion
Q=[B0; B1; B2; B3];
   
% Normalization Check
%[Q]=q_check(Q);

return;