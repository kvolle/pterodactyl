function [ Turb ] = TurbCalc( V,T,b,Lu,Lv,Lw,sigset,randset,TurbPast )
% Function to Calculate the Turbulence Components
%   The turbulence components are [ut;vt;wt;pt;qt;rt]

% Change Log
%{
    12/2012 - Function Written by Trevor Bennett
    5/15/2013 - Code Packaged by Trevor Bennett
%}

% White Noise: Unit Variance
% sign = 1;
% The term: *(sigu/sign)*randx = *sigu*randx

% Compensation for Negative Velocities
V1 = abs(V(1));
V2 = abs(V(2));
V3 = abs(V(3));

% TurbPast Contains the Previous Step Turbulence Data
utpast = TurbPast(1,1);
vtpast = TurbPast(2,1);
wtpast = TurbPast(3,1);
ptpast = TurbPast(4,1);
qtpast = TurbPast(5,1);
rtpast = TurbPast(6,1);

% Turbulence Realizations
ut = (1-V1*T/Lu)*utpast + sqrt(2*V1*T/Lu)*sigset(1)*randset(1);
vt = (1-V2*T/Lu)*vtpast + sqrt(2*V2*T/Lv)*sigset(2)*randset(2);
wt = (1-V3*T/Lu)*wtpast + sqrt(2*V3*T/Lw)*sigset(3)*randset(3);

% Rotational have +- on second term.  r set to (-),q set to (+)
pt = 0; % Current Place Holder
qt = (1-pi*V2*T/(4*b))*qtpast - (pi/(4*b))*(wt-wtpast);
rt = (1-pi*V3*T/(3*b))*rtpast - (pi/(3*b))*(vt-vtpast);

Turb = [ut;vt;wt;pt;qt;rt];

end

