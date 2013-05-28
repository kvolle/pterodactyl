% Function: [Q]=quatmult(Q1,Q2)
%
% Description: This function calculates quaternion obtained by
%              multiplying two different quaternions.  This is
%              the equivalent of combining two subsequent rotations.
%
% Inputs: [Q1],[Q2]=[Fist-quaternion],[Second_quaternion]
%
%         For example: (Q1,Q2)=([1,0,0,0]',[0,0,0,1]')
%
% Output: [Q] is the Quaternion obtained from the multiplication.
%
% Last Modified:
%     07/31/97 - Andres Mur-Dongil
%     02/15/98 - G. Chamitoff (Algorithm Correction - was Reversed)
%     05/01/98 - G. Chamitoff (Fast Version w/ Only Essential Checks)

function [Q]=quatmult(Q1,Q2)

% Checking Quaternion Validity
%Q1=q_check(Q1);
%Q2=q_check(Q2);

% First Rotation
Q1_mat=[Q1(1) -Q1(2) -Q1(3) -Q1(4); ...
        Q1(2)  Q1(1) -Q1(4)  Q1(3); ...
        Q1(3)  Q1(4)  Q1(1) -Q1(2); ...
        Q1(4) -Q1(3)  Q1(2)  Q1(1)];
 
% Product of the Rotations 
Q = Q1_mat*Q2;

% Checking normality of the quaternion
%Q=q_check(Q);

 
return;
