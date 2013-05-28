% Function: [eul]=ctm2eul(ctm,seq)
%
% Description: This program determines the Euler Angles for a given
%              rotation sequence that corresponds to the given
%              Coordinate Transformation Matrix.
%
% Input: [ctm] is the Coordinate Transformation Matrix. 
%
%        [seq] is a row vector that specifies the axis order for each 
%              successive rotation where
%                         1 = X Axis Rotation = Roll 
%                         2 = Y Axis Rotation = Pitch
%                         3 = Z Axis Rotation = Yaw
%              seq=[1st Axial Rotation, ...
%                   2nd Axial Rotation, ...
%                   3rd Axial Rotation]
%              For example: [seq]=[2 3 2] (Pitch, Yaw, Pitch)
%
% Output: [eul] is a row vector of the Euler Angles associated 
%               with the specific rotation (in degrees).
%
% Last Modified: 
%     06/19/96 - A. Mur-Dongil
%     02/10/97 - T. Carter
%     07/31/97 - A. Mur-Dongil
%     02/15/98 - G. Chamitoff (Algorithm Replaced Entirely to Handle
%                              Null Rotations, Singularity Conditions,
%                              and Orthogonal Rotations Properly.
%                              Euler Ambiquities Resolved by Adding
%                              An Additional Constraint that Theta3=0.
%     05/01/98 - G. Chamitoff (Fast Version w/ Only Essential Checks)
% 

function [eul]=ctm2eul(ctm_in,seq_in)

SEQ = [1 2 1; 1 2 3; 1 3 1; 1 3 2; 2 1 2; 2 1 3; 2 3 1; 2 3 2;...
       3 1 2; 3 1 3; 3 2 1; 3 2 3];  %These are all 12 variations

% Checking Orthonormality of the CTM
C=ctm_chck(ctm_in);

% Checking Rotation Sequence
%seq=seq_chck(seq_in);
seq=seq_in;

% Case by Case Computation 
if (seq==SEQ(1,:))      % [1 2 1]
    if (abs(C(1,1))<1.0)
        eul(1)=atan2(C(1,2),-C(1,3));
        eul(2)=atan2(real(sqrt(1-C(1,1)^2)),C(1,1));
        eul(3)=atan2(C(2,1),C(3,1));
    else
        eul(1)=atan2(C(2,3),C(2,2));
        eul(2)=acos(C(1,1));
        eul(3)=0;
    end;
elseif (seq==SEQ(2,:))  % [1 2 3]
    if (abs(C(3,1))<1.0)
        eul(1)=atan2(-C(3,2),C(3,3));
        eul(2)=atan2(C(3,1),real(sqrt(1-C(3,1)^2)));
        eul(3)=atan2(-C(2,1),C(1,1));
    else
        eul(1)=atan2(C(2,3),C(2,2));
        eul(2)=asin(C(3,1));
        eul(3)=0;
    end;
elseif (seq==SEQ(3,:))  % [1 3 1]
    if (abs(C(1,1))<1.0)
        eul(1)=atan2(C(1,3),C(1,2));
        eul(2)=atan2(real(sqrt(1-C(1,1)^2)),C(1,1));
        eul(3)=atan2(C(3,1),-C(2,1));
    else
        eul(1)=atan2(-C(3,2),C(3,3));
        eul(2)=acos(C(1,1));
        eul(3)=0;
    end;
elseif (seq==SEQ(4,:))  % [1 3 2]
    if (abs(C(2,1))<1.0)
        eul(1)=atan2(C(2,3),C(2,2));
        eul(2)=atan2(-C(2,1),real(sqrt(1-C(2,1)^2)));
        eul(3)=atan2(C(3,1),C(1,1));
    else
        eul(1)=atan2(-C(3,2),C(3,3));
        eul(2)=asin(-C(2,1));
        eul(3)=0;
    end;
elseif (seq==SEQ(5,:))  % [2 1 2]
    if (abs(C(2,2))<1.0)
        eul(1)=atan2(C(2,1),C(2,3));
        eul(2)=atan2(real(sqrt(1-C(2,2)^2)),C(2,2));
        eul(3)=atan2(C(1,2),-C(3,2));
    else
        eul(1)=atan2(-C(1,3),C(1,1));
        eul(2)=acos(C(2,2));
        eul(3)=0;
    end;
elseif (seq==SEQ(6,:))  % [2 1 3]
    if (abs(C(3,2))<1.0)
        eul(1)=atan2(C(3,1),C(3,3));
        eul(2)=atan2(-C(3,2),real(sqrt(1-C(3,2)^2)));
        eul(3)=atan2(C(1,2),C(2,2));
    else
        eul(1)=atan2(-C(1,3),C(1,1));
        eul(2)=asin(-C(3,2));
        eul(3)=0;
    end;
elseif (seq==SEQ(7,:))  % [2 3 1]
    if (abs(C(1,2))<1.0)
        eul(1)=atan2(-C(1,3),C(1,1));
        eul(2)=atan2(C(1,2),real(sqrt(1-C(1,2)^2)));
        eul(3)=atan2(-C(3,2),C(2,2));
    else
        eul(1)=atan2(C(3,1),C(3,3));
        eul(2)=asin(C(1,2));
        eul(3)=0;
    end;
elseif (seq==SEQ(8,:))  % [2 3 2]
    if (abs(C(2,2))<1.0)
        eul(1)=atan2(C(2,3),-C(2,1));
        eul(2)=atan2(real(sqrt(1-C(2,2)^2)),C(2,2));
        eul(3)=atan2(C(3,2),C(1,2));
    else
        eul(1)=atan2(C(3,1),C(3,3));
        eul(2)=acos(C(2,2));
        eul(3)=0;
    end;
elseif (seq==SEQ(9,:))  % [3 1 2]
    if (abs(C(2,3))<1.0)
        eul(1)=atan2(-C(2,1),C(2,2));
        eul(2)=atan2(C(2,3),real(sqrt(1-C(2,3)^2)));
        eul(3)=atan2(-C(1,3),C(3,3));
    else
        eul(1)=atan2(C(1,2),C(1,1));
        eul(2)=asin(C(2,3));
        eul(3)=0;
    end;
elseif (seq==SEQ(10,:)) % [3 1 3]
    if (abs(C(3,3))<1.0)
        eul(1)=atan2(C(3,1),-C(3,2));
        eul(2)=atan2(real(sqrt(1-C(3,3)^2)),C(3,3));
        eul(3)=atan2(C(1,3),C(2,3));
    else
        eul(1)=atan2(C(1,2),C(1,1));
        eul(2)=acos(C(3,3));
        eul(3)=0;
    end;
elseif (seq==SEQ(11,:)) % [3 2 1]
    if (abs(C(1,3))<1.0)
        eul(1)=atan2(C(1,2),C(1,1));
        eul(2)=atan2(-C(1,3),real(sqrt(1-C(1,3)^2)));
        eul(3)=atan2(C(2,3),C(3,3));
    else
        eul(1)=atan2(-C(2,1),C(2,2));
        eul(2)=asin(-C(1,3));
        eul(3)=0;
    end;
elseif (seq==SEQ(12,:)) % [3 2 3]    
    if (abs(C(3,3))<1.0)
        eul(1)=atan2(C(3,2),C(3,1));
        eul(2)=atan2(real(sqrt(1-C(3,3)^2)),C(3,3));
        eul(3)=atan2(C(2,3),-C(1,3));
    else
        eul(1)=atan2(-C(2,1),C(2,2));
        eul(2)=acos(C(3,3));
        eul(3)=0;
    end;
end;

% Converting the angles to degrees
eul=eul*180/pi;

return;
