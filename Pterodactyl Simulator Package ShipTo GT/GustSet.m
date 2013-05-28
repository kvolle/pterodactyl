function [ options ] = GustSet( varargin )
% GustSet: Alters the Default Inputs to DrydenGust
%   Possible Values for the DrydenGust Function. Default in {}:
%       Gust:
%           The current gust model version produces gusting winds in the 
%           plane parallel to the ground.
%       # Wind: 'Win' [ {16.8781} (10 Knots) , [Wind Speed] ]
%               Steady wind speed in ft/s.
%       # Gust On: 'GOn' [ {1} , 0 ]
%               Toggle on/off the contribution of Gusts. Selecting '0' for
%               this sets the GTo the value of wind to negate any GTo
%               effects.
%       # Gust To: 'GTo' [ {33.7562} (20 Knots) , [Gust To Speed] ]
%               Gust to wind speed in ft/s.
%       # Distribution: 'Dis' [ {Uniform} 'Uni', Normal 'Gau']
%               The default range for uniform is 0 to 360
%               The default range for normal is mean: 180, stddev: 10
%       # Range: 'Rng' [ {[0,360]} , [lower,upper] ]
%               The Range option only applies for the uniform distribution
%               option. The desired range should be typed in vector form.
%               The units of Range are degrees.
%               This option is required if 'Dis' 'Uni' is selected.
%       # Normal Distr. Set: 'NDS' [ {[180,10]} , [mean,stddev] ]
%               The 'NDS' option only applies for the normal distribution
%               option. The desired mean and standard deviation should be 
%               typed in vector form. The units of NDS are degrees.
%               This option is required if 'Dis' 'Gau' is selected.
%       # Gust Probability: 'Prb' [ {0.05} , [Probability] ]
%               Acceptable values of 'Prb' are in the closed interval [0,1]
%
%       Turbulence:
%           The current turbulence model produces 3-dimensional turbulence
%           in both translational and rotational air velocities. The
%           rotational air velocities are not as accurate.
%       # Turbulence On: 'TOn' [ {1} , 0 ]
%               Toggle on/off the contribution of Turbulence
%       # Altitude: 'Alt' [ {Low} 'Low', Normal 'Nor' , Variable 'Var'] 
%               Low Altitude is defined as below < 1000ft
%               Normal Altitude is defined as above (>) 2000ft
%               Variable Altitude checks at each time step
%                   * Use Variable if expected to operate between 1000-2000
%                   ft and/or plan to shift between altitude ranges.
%       # Turbulence: 'Tur' [ {Light} 'Lht', Moderate 'mod', Severe 'sev']
%               The Turbulence for Low Altitude is defined as:
%               Light = 15 knots, Moderate = 30 knots, Severe = 45 knots
%       # MIL: 'Mil' [ {MIL-HDBK-1797} 'HBK', MIL-F-8785C '-F-']
%               The gust formulation differs slightly between the two MIL
%       # Flight: 'Env' [ {Hover} 'Hov', Forward 'For']
%               The Hover condition is used when the forward body speed is
%               not significantly greater than velocity in the lateral or
%               vertical speeds. The scale length for the longitudinal
%               direction is set to be the scale length of the lateral
%               direction.
%               The Forward flight condition refers to the forward velocity
%               in the body x-axis.
%   Default Values can be altered using the GustSet Function
%
%   Options Strucuture -> ('input', 'value', 'input2', 'value2', ...)
%


% Change Log
%{
    12/2012 - Function Written by Trevor Bennett
    1/2013 - New Features and Data Passing Added
    5/15/2013 - Code Packaged by Trevor Bennett
%}


%% Options
% Default
options.WOn = 1;                % Winds Toggled On
options.Win = 16.8781;          % [ft/s]
options.GOn = 1;                % Gusts Toggled On
options.GTo = 33.7562;          % [ft/s]
options.Dis = 'Uni';
options.Rng = [0,360];          % Heading [degrees]
options.NDS = [180,10];         % [degrees]
options.Prb = 0.05;             % Should be multiplied by time step
options.TOn = 1;                % Turbulence Toggled On
options.Alt = 'Low';
options.Tur = 'Lht';
options.Mil = 'HBK';
options.Env = 'Hov';

% Set Options
n = nargin/2;

for i = 1:n
    name = varargin{1,2*i-1};
    value = varargin{1,2*i};
    switch name
        case {'Win','win'}
            options.Win = value;
        case {'GOn','Gon','gon'}
            options.GOn = value;
            if (value == 0)
                options.GTo = options.Win;
            end
        case {'GTo','Gto','gto'}
            options.GTo = value;
        case {'Dis','dis'}
            options.Dis = value;
        case {'Rng','rng'}
            options.Rng = value;
        case {'NDS','Nds','nds'}
            options.NDS = value;
        case {'Prb','prb'}
            options.Prb = value;
        case {'TOn','Ton','ton'}
            options.TOn = value;
        case {'Alt','alt'}
            options.Alt = value;
        case {'Tur','tur'}
            options.Tur = value;
        case {'Mil','mil'}
            options.Mil = value;
        case {'Env','env'}
            options.Env = value;
    end
end

