% Function: [Q]=q_check(Q_in)
%
% Description: This function checks if the input is a valid Quaternion.
%
% Input: [Q_in] is a Quaternion (a 4 x 1 column vector).
%
% Output: [Q] is the Quaternion after the validity check and any
%             corrections have been performed.
%
% Last Modified: 
%     07/28/97 - Andres Mur-Dongil
%     02/15/98 - G. Chamitoff (Protect Division by Zero)

function [Q]=q_check(Q_in)

Q_in = real(Q_in);

% Checking if the Quaternion has norm |Q|=1
if (real(norm(Q_in))<100*eps)
    disp('Message from Q_CHECK - Quaternion Near Zero - set to [1 0 0 0]');
    Q=[1 0 0 0]';
else
    Q=Q_in/real(norm(Q_in));
end;
if (Q(1)<0)  % Force Q1 to be Positive
    Q = -Q;  % DO NOT REMOVE - IMPORTANT FOR OTHER FUNCTIONS
end
 
return; 
