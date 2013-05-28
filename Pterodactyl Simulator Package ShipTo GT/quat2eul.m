% Function: [eul]=quat2eul(seq,Q)
%
% Description: This program determines the Euler Angle set for a given
%              given Rotation Sequence that corresponds to the given
%              Quaternion.
%
% Inputs:[seq] is a row vector that specifies the axis order for each 
%              successive rotation where
%                         1 = X Axis Rotation = Roll 
%                         2 = Y Axis Rotation = Pitch
%                         3 = Z Axis Rotation = Yaw
%              seq=[1st Axial Rotation, ...
%                   2nd Axial Rotation, ...
%                   3rd Axial Rotation]
%              For example: [seq]=[2 3 2] (Pitch, Yaw, Pitch)
%        [Q] is the Quaternion (a 4 x 1 column vector).
%
% Output:[eul] is the Euler Angles associated with the given Quaternion 
%         expressed in the desired sequence.  
%
% Last Modified: 
%     06/19/96 - A. Mur-Dongil
%     02/10/97 - T. Carter
%     07/14/97 - A. Mur-Dongil
%     02/15/98 - G. Chamitoff (Comments Changed, Checks Moved to Subs)
% 

function [eul]=quat2eul(seq,Q)

% Range Checks Provided by Subs

% Generating the Coordinate Transformation Matrix from the Quaternion  
ctm = quat2ctm(Q);

% Generating the Euler Angles from the Coordinate Transformation Matrix
% and the Rotation Sequence
[eul] = ctm2eul(ctm,seq);
  
return;