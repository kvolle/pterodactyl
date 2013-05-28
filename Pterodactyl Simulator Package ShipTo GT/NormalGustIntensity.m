function [ sigu ] = NormalGustIntensity( z, TurIn )
% Function to Perform Lookup For Gust Intensity
%   The tabluated values are extracted by inspection of the probability and
%   intensity versus altitude. The plot can be found in the MIL-HDBK-1797
%   and MIL-F-8785C documentation.
%       The data was acquired through inspection and is accurate to 2
%       decimal places.

% Change Log
%{
    12/2012 - Function Written by Trevor Bennett
    5/15/2013 - Code Packaged by Trevor Bennett
%}

%% Persistent Variables
persistent Alt Int IntvAlt

if isempty(Alt)
    % Altitude in [ft]
    Alt = [2000,4000,8000,10000,15000,20000,25000,30000,35000,40000,45000,...
        50000,55000,60000,65000];
    
    % Intensity Possibilities ['Lht','Mod','Sev']
    Int = [1,2,3];
    
    % Data for Intensity versus Altitude [ft/s]
    IntvAlt = ...
   [6.6700    8.3300   15.3500
    7.3300   10.5000   23.0000
    6.6700   10.0000   23.6700
    6.0000    9.3300   23.3300
    4.6700    8.0000   22.1700
    3.6700    7.3300   21.0000
    2.6700    6.6700   20.0000
    1.5000    6.0000   18.0000
    0.3300    5.0000   16.0000
    0.1500    4.6600   15.5000
         0    4.1700   15.0000
         0    3.5000   13.6700
         0    2.8300   12.0000
         0    1.3300   10.0000
         0         0    8.0000];
end

switch TurIn
    case {'Lht','lht'}
        Tur = 1;
    case {'Mod','mod'}
        Tur = 2;
    case {'Sev','sev'}
        Tur = 3;
end

sigu = interp2(Int,Alt,IntvAlt,Tur,(-z));

end

