function [ Gust ] = DrydenGust( V,z,b,wn,C,T,varargin )
% DrydenGust: Generates a Gust Vector Using the Dryden Gust Model.
%   The DrydenGust Function encampsulates variations in Altitude,
%   Turbulence Intensity, MIL HDBK Version, Hover vs. Forward Flight
%   through stochastic processes.
%   DrydenGust is written for Imperial Units.
%   Input Variables for DrydenGust
%       # V: Vector of u,v,w velocities. [ft/s]
%       # z: Altitude (Negative - Z Axis Down) [ft]
%       # b: Wing Span [ft]
%       # wn: Natural Frequency of Aircraft [rad/s]
%       # C: Rotation Matrix from Inertial Frame to Body Frame
%       # T: Integration Step Size [sec]
%       # Options: Listed Below:
%
%   Possible Values for the DrydenGust Function. Default in {}:
%       Gust:
%           The current gust model version produces gusting winds in the 
%           plane parallel to the ground.
%       # Wind On: 'WOn' [ {1} , 0 ]
%               Toggle on/off the contribution of Wind
%       # Wind: 'Win' [ {16.8781} (10 Knots) , [Wind Speed] ]
%               Steady wind speed in ft/s.
%       # Gust On: 'GOn' [ {1} , 0 ]
%               Toggle on/off the contribution of Gusts
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
%   Gust Model Data Storage:
%       The Dryden Gust Function was originally written to be compatable
%       with a 4th Order Runge Kutta Solver. The RK4 only requires data to
%       be recorded the first time the function is called to be paired with
%       recorded states. The last state will not have wind data associated
%       with it because of the storage method.
%       # The wind profiles are saved to the global variable Wind_Profile.
%       # The varaible "RKorder" controls when the data is written to 
%       Wind_Profile.
%
%   Gust Model Output Structure
%       Summing the Gust Wave and Turbulence Outputs gives the Gust Vector
%       Output of the form Gust = [ug;vg;wg;pg;qg;rg]
%       Where u,v,w are velocities, p,q,r are rates  in the body frame.
%       The Turbulence is updated on the last step of the RKorder
%

% Change Log
%{
    12/2012 - Function Written by Trevor Bennett
    1/2013 - New Features and Data Passing Added
    5/15/2013 - Code Packaged by Trevor Bennett
%}

%% Persistent Variables %%
global Wind_Profile

persistent WOn Win GOn GTo Dis Rng NDS Prb TOn Alt Tur Mil Env
persistent GustFlag HeadingRot WaveTime WaveCheck
persistent randset lW20 mW20 sW20 
persistent TurbPast counter RKorder k

%% Initialize All Persistent Variables %%
if isempty(counter)
    % Initialize Counter
    counter = 1;
    k = 1;
    GustFlag = 0;
    WaveTime = 0;
    WaveCheck = zeros(3,1);
    
    % Code Written for Fourth Order Runge Kutta
    RKorder = 4;
    
    % Default Turbulence Data
    lW20 = 15*1.68781;      % Intensity of Light Wind at 20ft [knots -> ft/s]
    mW20 = 30*1.68781;      % Intensity of Moderate Wind at 20ft [knots -> ft/s]
    sW20 = 45*1.68781;      % Intensity of Severe Wind at 20ft [knots -> ft/s]
    
    % Default Initial Turbulence Values
    TurbPast = zeros(6,1);
end

%% Variable and Options Initialization %%
n = nargin;
if isempty(Win)
    if (n==7)
        options = varargin{1,1};
        WOn = options.WOn;
        Win = options.Win;
        GOn = options.GOn;
        GTo = options.GTo;
        Dis = options.Dis;
        Rng = options.Rng;
        NDS = options.NDS;
        Prb = options.Prb;
        TOn = options.TOn;
        Alt = options.Alt;
        Tur = options.Tur;
        Mil = options.Mil;
        Env = options.Env;
    elseif isempty(Win)
        WOn = 1;
        Win = 16.8781;
        GOn = 1;
        GTo = 33.7562;
        Dis = 'Uni';
        Rng = [0,360];
        NDS = [180,10];
        Prb = 0.05;
        TOn = 1;
        Alt = 'Low';
        Tur = 'Lht';
        Mil = 'HBK';
        Env = 'Hov';
    end
end

%% Variable Altitude Set %%
if ((strcmp(Alt,'Var') == 1)||(strcmp(Alt,'var') == 1))
    if(abs(z) <= 1000)
        Alt = 'Low';
    elseif(abs(z) >= 2000)
        Alt = 'Nor';
    else
        Alt = 'Mid';
    end
end   

%% Gust Wave %%
% Each Time Step Assess Wind Heading
if (counter == 1)
    switch Dis
        case {'Uni','uni'}
            Heading = Rng(1) + (Rng(2) - Rng(1))*rand;
        case {'Gau','gau'}
            Heading = NDS(1) + NDS(2)*randn;
    end
    HeadingRot = [-cosd(Heading) sind(Heading) 0;
                  -sind(Heading) -cosd(Heading) 0;
                  0 0 1];
end

if (GOn == 1)
    % Probability of Gust Occuring on this Time Step
    if (GustFlag == 0)
        wavemag = 0;
        if (counter == 1)
            num = rand;
            if (num <= Prb)
                GustFlag = 1;

                % Time Checkpoints
                tm = pi/wn;                     % Rise time of 1-cos wave
                tdur = tm;                      % Time duration at peak strength
                WaveCheck(1,1) = tm;            % Time ellapsed to peak strength    
                WaveCheck(2,1) = tm + tdur;     % Time ellapsed at end of peak strength
                WaveCheck(3,1) = 2*tm + tdur;   % Time ellapsed at save disappearance
                WaveTime = 0;
            end
        end
    end
    % Gust Wave Calculation
    if (GustFlag == 1)
        % Compute Velocity Increment from the difference in wind and GTo
        velinc = GTo - Win;
        if (WaveTime < WaveCheck(1))
            wavemag = velinc*(1 - cos(wn*WaveTime))/2;
        elseif (WaveTime < WaveCheck(2))
            wavemag = velinc;        
        elseif (WaveTime < WaveCheck(3))
            wavemag = velinc*(1 + cos(wn*WaveTime))/2;
        else
            GustFlag = 0;
            wavemag = 0;
        end
    end
end

if (WOn == 1)
    if (GOn == 1)
        % Rotate Wind Into Body Axis, Sum with Gust Wave
        Wind = C*HeadingRot*[(wavemag + Win);0;0]; 

        % Advance Gust WaveTime
        if (counter == RKorder)
            WaveTime = WaveTime + T;
        end
    else
        % Rotate Wind Into Body Axis, Sum with Gust Wave
        Wind = C*HeadingRot*[Win;0;0]; 

        % Advance Gust WaveTime
        if (counter == RKorder)
            WaveTime = WaveTime + T;
        end
    end
else
    if (GOn == 1)
        % Rotate Wind Into Body Axis, Sum with Gust Wave
        Wind = C*HeadingRot*[wavemag;0;0]; 

        % Advance Gust WaveTime
        if (counter == RKorder)
            WaveTime = WaveTime + T;
        end
    else
        Wind = [0;0;0];
    end
end

%% Generation of Random Variables for Turbulence Modeling %%
%   Occurs on the first of RKorder function calls
if (counter == 1)
    randset(1,1) = randn;
    randset(2,1) = randn;
    randset(3,1) = randn;
end     

%% Turbulence Intensity Calculation %%
switch Alt
    case {'Low','low'}
        switch Tur
            case {'Lht','lht'}
                sigw = 0.1*lW20;
            case {'Mod','mod'}
                sigw = 0.1*mW20;
            case {'Sev','sev'}
                sigw = 0.1*sW20;
        end
        sigu = sigw/((0.177 + 0.000823*(-z))^0.4);
        sigv = sigu;
        sigset = [sigu;sigv;sigw];
    case {'Nor','nor'}
        sigu = NormalGustIntensity(z,Tur);
        sigv = sigu;
        sigw = sigu;
        sigset = [sigu;sigv;sigw];
    case 'Mid'
        % Calculate Lower Interpolation Values
        switch Tur
            case {'Lht','lht'}
                lsigw = 0.1*lW20;
            case {'Mod','mod'}
                lsigw = 0.1*mW20;
            case {'Sev','sev'}
                lsigw = 0.1*sW20;
        end
        lsigu = lsigw/((0.177 + 0.000823*(1000))^0.4);
        lsigv = lsigu;
        lsigset = [lsigu;lsigv;lsigw];
        % Calculate Upper Interpolation Values
        nsigu = NormalGustIntensity(-2000,Tur);
        nsigv = nsigu;
        nsigw = nsigu;
        nsigset = [nsigu;nsigv;nsigw];
end

%% Turbulence Calculation %%
switch Mil
    % Using the MIL-HDBK-1797 Specifications 
    case {'HBK','Hbk','hbk'}
        switch Alt
            case {'Low','low'}
                switch Env
                    case {'Hov','hov'}
                        Lu = (-z)/((0.177 + 0.000823*(-z))^1.2)/2;
                        Lv = Lu;
                        Lw = (-z)/2;
                        turbtemp = TurbCalc(V,T,b,Lu,Lv,Lw,sigset,randset,TurbPast);
                        Turb(1:3,1) = C*HeadingRot*turbtemp(1:3,1);
                        Turb(4:6,1) = C*HeadingRot*turbtemp(4:6,1);
                    case {'For','for'}
                        Lu = (-z)/((0.177 + 0.000823*(-z))^1.2);
                        Lv = Lu/2;
                        Lw = (-z)/2;
                        turbtemp = TurbCalc(V,T,b,Lu,Lv,Lw,sigset,randset,TurbPast);
                        Turb(1:3,1) = C*HeadingRot*turbtemp(1:3,1);
                        Turb(4:6,1) = C*HeadingRot*turbtemp(4:6,1);
                end
            case {'Nor','nor'}
                % For medium to high altitudes the turbulence scale lengths
                % and intensities are based on the assumption that the 
                % turbulence is isotropic
                switch Env
                    case {'Hov','hov'}
                        Lu = 1750/2;
                        Lv = Lu;
                        Lw = Lu;
                        Turb = TurbCalc(V,T,b,Lu,Lv,Lw,sigset,randset,TurbPast);
                    case {'For','for'}
                        Lu = 1750;
                        Lv = 1750/2;
                        Lw = Lv;
                        Turb = TurbCalc(V,T,b,Lu,Lv,Lw,sigset,randset,TurbPast);
                end
            case 'Mid'
                switch Env
                    case {'Hov','hov'}
                        lLu = (-z)/((0.177 + 0.000823*(-z))^1.2)/2;
                        lLv = lLu;
                        lLw = (-z)/2;
                        nLu = 1750/2;
                        nLv = nLu;
                        nLw = nLu;
                        turbtemp = TurbCalc(V,T,b,lLu,lLv,lLw,lsigset,randset,TurbPast);
                        lTurb(1:3,1) = C*HeadingRot*turbtemp(1:3,1);
                        lTurb(4:6,1) = C*HeadingRot*turbtemp(4:6,1);
                        nTurb = TurbCalc(V,T,b,nLu,nLv,nLw,nsigset,randset,TurbPast);
                        Turb = zeros(6,1);
                        for i = 1:6
                            Turb(i) = interp1([-1000,-2000],[lTurb(i),nTurb(i)],z);
                        end
                    case {'For','for'}
                        lLu = (-z)/((0.177 + 0.000823*(-z))^1.2);
                        lLv = lLu/2;
                        lLw = (-z)/2;
                        nLu = 1750;
                        nLv = 1750/2;
                        nLw = nLv;
                        turbtemp = TurbCalc(V,T,b,lLu,lLv,lLw,lsigset,randset,TurbPast);
                        lTurb(1:3,1) = C*HeadingRot*turbtemp(1:3,1);
                        lTurb(4:6,1) = C*HeadingRot*turbtemp(4:6,1);
                        nTurb = TurbCalc(V,T,b,nLu,nLv,nLw,nsigset,randset,TurbPast);
                        Turb = zeros(6,1);
                        for i = 1:6
                            Turb(i) = interp1([-1000,-2000],[lTurb(i),nTurb(i)],z);
                        end
                end              
        end 
        
    % Using the MIL-F-8785C Specifications    
    case {'-F-','-f-'}
        switch Alt
            case {'Low','low'}
                % Constants are equivalent for 'Hov' and 'For' Conditions
                Lu = (-z)/((0.177 + 0.000823*(-z))^1.2);
                Lv = Lu;
                Lw = (-z);
                turbtemp = TurbCalc(V,T,b,Lu,Lv,Lw,sigset,randset,TurbPast);
                Turb(1:3,1) = C*HeadingRot*turbtemp(1:3,1);
                Turb(4:6,1) = C*HeadingRot*turbtemp(4:6,1);
            case {'Nor','nor'}
                % For medium to high altitudes the turbulence scale lengths
                % and intensities are based on the assumption that the 
                % turbulence is isotropic
                % Constants are equivalent for 'Hov' and 'For' Conditions
                Lu = 1750;
                Lv = Lu;
                Lw = Lu;
                Turb = TurbCalc(V,T,b,Lu,Lv,Lw,sigset,randset,TurbPast);
            case 'Mid'
                lLu = (-z)/((0.177 + 0.000823*(-z))^1.2);
                lLv = lLu;
                lLw = (-z);
                nLu = 1750;
                nLv = nLu;
                nLw = nLu;
                turbtemp = TurbCalc(V,T,b,lLu,lLv,lLw,lsigset,randset,TurbPast);
                lTurb(1:3,1) = C*HeadingRot*turbtemp(1:3,1);
                lTurb(4:6,1) = C*HeadingRot*turbtemp(4:6,1);
                nTurb = TurbCalc(V,T,b,nLu,nLv,nLw,nsigset,randset,TurbPast);
                Turb = zeros(6,1);
                for i = 1:6
                    Turb(i) = interp1([-1000,-2000],[lTurb(i),nTurb(i)],z);
                end 
        end
end

%% Data Storage and Pass-Out
Gust(1:3,1) = Turb(1:3,1) + Wind;            
Gust(4:6,1) = Turb(4:6,1);

% Past Values Assigned on the First Call of the Runge Kutta
if (counter == 1)
    % Store Wind_Profile
    Wind_Profile(k,1:6) = Gust;
    k = k + 1;
    
    % Increment Counter
    counter = counter + 1;
elseif (counter == RKorder)
    % Update TurbPast
    TurbPast = Turb;
    % Reset Counter
    counter = 1;
else
    % Increment Counter
    counter = counter + 1;
end

end



