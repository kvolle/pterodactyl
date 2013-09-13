%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                        Pterodactyl Simulator                            %
%                              Version 5                                  %
%                           Trevor Bennett                                %
%                      Graduate Research Project                          %
%                                                                         %
%                              Main Code                                  %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Change Log
%{
    8/2012 - Code Written by Trevor Bennett
    5/15/2013 - Code Packaged by Trevor Bennett
%}

% General Notes
%{
    * The Pterodactyl aircraft properties can be edited in the 
        AircraftProperties Function.
    * The unit system is English [ft, lbs, s].
    * The simulator uses quaternions to describe the orientation of the 
        body frame relative to the inertail frame.
    * The Inertial Frame is described as:
        X: Aligned with True North
        Y: Right Handed Coordinate System with X and Z
        Z: Down (Into the Earth)
    * First Version to incorporate active controls

%}

% Versions
%{
    * Version 1 is the exploratory development of the Pterodactyl
    simulator. It runs an incomplete simulation environment.

    * Version 2 is the first complete simulation environment for the
    Pterodactyl. Version 2 incorporates the Dryden Gust Model and online
    thrust modeling.

    * Version 3 builds upon Version 2 by restructing the passing of
    geometry information. Version 3 includes an offline interpolation of
    aerodynamic data to decrease online run time.

    * Version 4 speeds up Version 3 through several means:
        > Cross product function calls removed, now hardcoded.
        > Gust model includes toggles to exclude unecessary computation.
        > Simulation loop restructured to "sim forward" between control
            updates.
        > Curve fits the BEMT thrust and torque models. This can be used
            for control and/or speeding up runtime.
        > Includes controller plugin (MATLAB function file) to design
            controllers.
        
    * Version 5 builds on Version 4 by adding in Aileron Modeling and
        Control.
            > The Control Surfaces do not follow aircraft conventions. By 
                convention, positive deflection represents a positive 
                moment about the body axis. The deflection angles for 
                control surfaces in this code is that a positive deflection
                represents a positive change in angle of attack for the
                airfoil in the airfoil fixed frame.
%}

% To Be Done
%{
          
    * Control Plot
        > Use steps() for correct control plot

    * Data Passing using Structures instead of Global

%}

% close all;
clear all;

%function SimulatorMain()

%% Pterodactyl Properties %%
% Initialize Plot Number
global plotNum
plotNum = 1;
% Initialization Functions
PterodactylGeometry;
%BEMTfit = BEMT_ControlCoeffs;

%% Initial Conditions %%
% Aircraft State Vector [x; y; z; Beta0; Beta1; Beta2; Beta3; u; v; w; p; q; r]
% Body X Aligned with Negative Z
CHov = [0 0 -1; 0 1 0;1 0 0];  

% Ptero Pitched Up
pitchang = 90*pi/180;
CPitch = [cos(pitchang) 0 -sin(pitchang);
          0 1 0;
          sin(pitchang) 0 cos(pitchang)];

CInit = CPitch;

% Beta is the Quaternion
Beta = q_check(ctm2quat(CInit));
yinit = [0; 0; -100; Beta(1); Beta(2); Beta(3); Beta(4); 0; 0; 0; 0; 0.5; 0];

%% Simulation Loop
global Wind_Profile t_step Control optionsDG

%%% Update Intervals %%%
%========================================================================%
% Simulator Step Size
t_max = 3.0;             % [s]
t_step = 0.01;
datanum = t_max/t_step + 1;

% Controller Update Frequency -> Step Size
CUfreq = 20;             % Hz
if ((1/CUfreq) < t_step)
    errstr1 = sprintf('The control update time was entered as %d.\n The simulation time step is %d.\n The control update has been altered to equal the simulation time step.\n',1/CUfreq,t_step);
    disp(errstr1)
    CUstep = t_step;
else
    step = 1/CUfreq;
    R = rem(step,t_step);
    if (R ~= 0)
        old = step;
        CUstep = ceil(step/t_step);   % The control update occures every CUstep*t_step
        errstr2 = sprintf('The control update time was set to %d from %d\n',CUstep,old);
        disp(errstr2)
    else
        CUstep = step;
    end            
end

RK_Steps = round(CUstep/t_step);
%========================================================================%

% Allocate Data Storage
Flight_Data = zeros(datanum,14);
Wind_Profile = zeros(datanum,6);
Control.RPM = zeros(datanum-1,3);
Control.Thrust = zeros(datanum-1,3);
Control.BodyForce = zeros(datanum-1,3);
Control.Torque = zeros(datanum-1,3);
Control.BodyTorque = zeros(datanum-1,3);

% Store Initial Conditions
t = 0;
Flight_Data(1,1) = t;
Flight_Data(1,2:14) = yinit;
CMD = PterodactylControl(yinit);
Control.RPM(1,1:3) = CMD.Omega*30/pi;
yn = yinit;
ind = 2;

%%% Simulation Loop %%%
profile on

% Wind Profile
% DrydenGust Model ON
% options = GustSet('Dis','Gau','NDS',[180,1]);
% DrydenGust Model OFF
optionsDG = GustSet('Win',0,'GOn',0,'TOn',0);

% Simulation
eps = 0.0001;
while (t <= t_max+eps)
    % Update Control
    CMD = PterodactylControl(yn);
    for step = 1:RK_Steps
        % Update Time
        t = t + t_step;
        if (t > t_max)
            continue;
        end
        
        % Runge Kutta
        ynplus = PterodactylRK4(t_step, yn, CMD);

        % Store Flight Data
        Flight_Data(ind,1) = t;
        Flight_Data(ind,2:14) = ynplus;
        % Store Control
        Control.RPM(ind,1:3) = CMD.Omega*30/pi;    % Convert to rpm
        
        % Prepare for next step
        yn = ynplus;
        ind = ind + 1;
    end
end
% Remove Extra Line
Flight_Data = Flight_Data(1:end-1,:);
Control.RPM = Control.RPM(1:end-1,:);
profile off
profile viewer

%% Euler Angles %%
Eul = zeros(datanum-1,3);
for i = 1:datanum-1
    q = [Flight_Data(i,5);Flight_Data(i,6);Flight_Data(i,7);Flight_Data(i,8)];
    Eul(i,:) = quat2eul([3,2,1],q);
end


%% Plotter %%
% Plot XYZ
figure (plotNum)
plotNum = plotNum + 1;
subplot(3,2,[1,3,5])
plot3(Flight_Data(:,2),Flight_Data(:,3),Flight_Data(:,4))
title('Motion of CG in Aircraft Inertial Frame. X Points North, Y East, Z Down')
set(gca,'zdir','reverse','ydir','reverse')
xlabel('X [ft]')
ylabel('Y [ft]')
zlabel('Z [ft]')
subplot(3,2,2)
plot(Flight_Data(:,1),Flight_Data(:,2))
title('X Location')
xlabel('Time [s]')
ylabel('X [ft]')
subplot(3,2,4)
plot(Flight_Data(:,1),Flight_Data(:,3))
title('Y Location')
xlabel('Time [s]')
ylabel('Y [ft]')
subplot(3,2,6)
plot(Flight_Data(:,1),Flight_Data(:,4))
title('Altitude')
set(gca,'ydir','reverse')
xlabel('Time [s]')
ylabel('Z [ft]')

%======================================================%
% Angular Velocities and Euler Angles
figure (plotNum)
plotNum = plotNum + 1;
subplot(3,2,1)
plot(Flight_Data(:,1),(180/pi)*Flight_Data(:,12))
title('Body X-Axis Rate, p')
xlabel('Time [s]')
ylabel('Angular Rate [deg/s]')
subplot(3,2,3)
plot(Flight_Data(:,1),(180/pi)*Flight_Data(:,13))
title('Body Y-Axis Rate, q')
xlabel('Time [s]')
ylabel('Angular Rate [deg/s]')
subplot(3,2,5)
plot(Flight_Data(:,1),(180/pi)*Flight_Data(:,14))
title('Body Z-Axis Rate, r')
xlabel('Time [s]')
ylabel('Angular Rate [deg/s]')
subplot(3,2,6)
plot(Flight_Data(:,1),Eul(:,1))
title('Yaw Angle of Pterodactyl')
xlabel('Time [s]')
ylabel('\psi [degrees]')
subplot(3,2,4)
plot(Flight_Data(:,1),Eul(:,2))
title('Pitch Angle of Pterodactyl')
xlabel('Time [s]')
ylabel('\theta [degrees]')
subplot(3,2,2)
plot(Flight_Data(:,1),Eul(:,3))
title('Roll Angle of Pterodactyl')
xlabel('Time [s]')
ylabel('\phi [degrees]')

%======================================================%
% Body Frame Linear Velocities
figure (plotNum)
plotNum = plotNum + 1;
subplot(3,1,1)
plot(Flight_Data(:,1),Flight_Data(:,9))
title('Body X-Axis Velocity, u')
xlabel('Time [s]')
ylabel('Velocity [ft/s]')
subplot(3,1,2)
plot(Flight_Data(:,1),Flight_Data(:,10))
title('Body Y-Axis Velocity, v')
xlabel('Time [s]')
ylabel('Velocity [ft/s]')
subplot(3,1,3)
plot(Flight_Data(:,1),Flight_Data(:,11))
title('Body Z-Axis Velocity, w')
xlabel('Time [s]')
ylabel('Velocity [ft/s]')

%======================================================%
% Motor Commands, Torques, and Forces
figure (plotNum)
plotNum = plotNum + 1;
subplot(3,2,[1,2])
plot(Flight_Data(:,1),Control.RPM(:,1),Flight_Data(:,1),Control.RPM(:,2),...
    Flight_Data(:,1),Control.RPM(:,3));
title('Commanded Motor Angular Velocities')
legend('Left Wing Motor','Center Wing Motor','Right Wing Motor')
xlabel('Time [s]')
ylabel('Control [rpm]')
subplot(3,2,3)
plot(Flight_Data(:,1),Control.Thrust(:,1),Flight_Data(:,1),Control.Thrust(:,2),...
    Flight_Data(:,1),Control.Thrust(:,3));
title('BEMT Thrust Produced, Magnitudes')
legend('Left Wing Motor','Center Wing Motor','Right Wing Motor')
xlabel('Time [s]')
ylabel('Thrust [lbsf]')
subplot(3,2,4)
plot(Flight_Data(:,1),Control.BodyForce(:,1),Flight_Data(:,1),Control.BodyForce(:,2),...
    Flight_Data(:,1),Control.BodyForce(:,3));
title('BEMT Thrust Forces in Body Frame')
legend('F_X','F_Y','F_Z')
xlabel('Time [s]')
ylabel('Thrust [lbsf]')
subplot(3,2,5)
plot(Flight_Data(:,1),Control.Torque(:,1),Flight_Data(:,1),Control.Torque(:,2),...
    Flight_Data(:,1),Control.Torque(:,3));
title('Motor Torque Magnitudes Acting on Pterodactyl')
legend('Left Wing Motor','Center Wing Motor','Right Wing Motor')
xlabel('Time [s]')
ylabel('Torque [ft-lbsf]')
subplot(3,2,6)
plot(Flight_Data(:,1),Control.BodyTorque(:,1),Flight_Data(:,1),Control.BodyTorque(:,2),...
    Flight_Data(:,1),Control.BodyTorque(:,3));
title('Motor Torque Acting on Pterodactyl in Body Frame')
legend('Q_X','Q_Y','Q_Z')
xlabel('Time [s]')
ylabel('Torque [ft-lbsf]')

%======================================================%
% Wind Profiles
if ((optionsDG.WOn ~= 0)&&(optionsDG.GOn ~= 0)&&(optionsDG.TOn ~= 0))
    figure (plotNum)
    plotNum = plotNum + 1;
    subplot(3,1,1)
    plot(Flight_Data(:,1),Wind_Profile(:,1))
    title('Wind Velocity in Body X-Direction')
    subplot(3,1,2)
    plot(Flight_Data(:,1),Wind_Profile(:,2))
    title('Wind Velocity in Body Y-Direction')
    subplot(3,1,3)
    plot(Flight_Data(:,1),Wind_Profile(:,3))
    title('Wind Velocity in Body Z-Direction')
end
