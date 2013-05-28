function [ f ] = BEMT_ControlCoeffs()
% BEMT_ControlCoeffs Generates the Control Mapping between desired thrust
% and the Commanded Motor Rotation Rate.
%   * BEMT Input Varialbles: Motor Rotation Rate, Velocity of Hub
%   * BEMT Output Variables: Thrust
%  > This function generates an equation that takes Thrust and Velocity of
%  the hub as input arguements and returns Commanded Motor Rotation Rate

% Change Log
%{
    2/2013 - Function Written by Trevor Bennett
    5/15/2013 - Code Packaged by Trevor Bennett
%}


% Global Variables
global BladeR Bladex BladeCLAlpha BladeCD0 BladeCL0
global BladeN BladePitch BladeChord plotNum

% Altititude
z = -20;
rho = 0.0023784722 * (1.0 + 6.8789e-6*z)^(4.258);

% Greatest Negative Free Stream Velocity
vInfMin = -20;
% Greatest Positive Free Stream Velocity
vInfMax = 50;
% Freestream Velocity into the Hub (Set)
minval = sqrt(-vInfMin);
maxval = sqrt(vInfMax);
stepsize = 0.5;
valslow = linspace(-minval,0,ceil(minval/stepsize));
valshigh = linspace(stepsize,maxval,ceil((maxval/stepsize)-1));
vInfSet = [-(valslow).^2,valshigh.^2]';
nCurves = length(vInfSet);

% Commanded Motor Rotation Rate (Set) [rad/s]
maxRPM = 12000;
minRPM = 3000;
RPMinc = 100;
rotRateCMDSet = linspace(minRPM*2*pi/60,maxRPM*2*pi/60,(maxRPM-minRPM)/RPMinc)';
nCMDs = length(rotRateCMDSet);

% Allocate Storage
ThrustMatrix = zeros(nCMDs,nCurves); 
ThrustVectors = zeros(nCMDs*nCurves,3);
TorqueMatrix = zeros(nCMDs,nCurves);
TorqueVectors = zeros(nCMDs*nCurves,3);
b_ind = length(Bladex);


for xind = 1:nCMDs
    % Commanded Rotation Rate
    Omega = rotRateCMDSet(xind,1);
    
    for yind = 1:nCurves
        % Free Stream Velocity at Hub [ft/s]
        vInf = vInfSet(yind,1);
        Vclimb = vInf;
        
        % Assuming Constant Vclimb for all Annuli of a Propeller
        lambdac = Vclimb/(Omega*BladeR);

        % Zero Out Integration Indicies and Temp
        Ttemp = 0; 
        Qtemp = 0;
        T1 = [0,0];             % [x coord, T @ x]
        Q1 = [0,0];             % [x coord, Q @ x]

        for j = 1:b_ind
            % Position Data
            x = Bladex(j);
            r = BladeR*x;
            c = BladeChord(j);
            Cla = BladeCLAlpha(j);
            Cd0 = BladeCD0(j);
            theta = BladePitch(j);

            % Compute Lambda 
            sigma = BladeN*c/(pi*BladeR);

            lambda = -(sigma*Cla/16 - lambdac/2) + ...
                sqrt((sigma*Cla/16 - lambdac/2)^2 + sigma*Cla*theta*x/8);

            % Compute Induced Velocity
            w = Omega*BladeR*(lambda - lambdac);

            % Numerical Integration for T and Q
            vterm = Vclimb + w;
            rterm = Omega*r;
            Cl = Cla*(theta - vterm/rterm) + BladeCL0;

            T2(1,1) = x;
            T2(1,2) = c*rterm*(rterm*Cl - vterm*Cd0);
            Ttemp = Ttemp + BladeR*(x - T1(1,1))*(T1(1,2) + T2(1,2))/2;
            T1 = T2;

            Q2(1,1) = x;
            Q2(1,2) = c*rterm*(rterm*r*Cd0 + vterm*Cl*r);
            Qtemp = Qtemp + BladeR*(x - Q1(1,1))*(Q1(1,2) + Q2(1,2))/2;
            Q1 = Q2;
        end
        % Compute Integral and Store
        ThrustVectors((xind-1)*nCurves + yind,1) = Omega;
        ThrustVectors((xind-1)*nCurves + yind,2) = Vclimb;
        ThrustVectors((xind-1)*nCurves + yind,3) = BladeN*rho*Ttemp/2;
        ThrustMatrix(xind,yind) = BladeN*rho*Ttemp/2;
        TorqueVectors((xind-1)*nCurves + yind,1) = Omega;
        TorqueVectors((xind-1)*nCurves + yind,2) = Vclimb;
        TorqueVectors((xind-1)*nCurves + yind,3) = BladeN*rho*Qtemp/2;
        TorqueMatrix(xind,yind) = BladeN*rho*Qtemp/2;
        
    end
end

%%% Development Plots - Thrust and Torque %%%

% figure (plotNum)
% plotNum = plotNum + 1;
% surf(vInfSet,rotRateCMDSet,ThrustMatrix) 
% tstr = sprintf('Thrust Produced at Given Motor Angular Velocity and Flow into the Disk\nAlt = %3.0f; Propeller: %2.0f x %1.0f\n',-z,BladeR*24,8);
% title(tstr)
% ylabel('Commanded Angular Velocity of the Propeller [rad/s]')
% xlabel('Disk Climb Velocity [ft/s]')
% zlabel('Thrust [lbsf]')
% 
% figure (plotNum)
% plotNum = plotNum + 1;
% surf(vInfSet,rotRateCMDSet,TorqueMatrix) 
% title('Torque Produced at Given Motor Angular Velocity and Flow into the Disk')
% ylabel('Commanded Angular Velocity of the Propeller [rad/s]')
% xlabel('Disk Climb Velocity [ft/s]')
% zlabel('Torque [lbsf*ft]')


%% Surface Fit %%
% Thrust Curve Fit Parameters
omegaPower = 2;
vPower = 2;
fittype = sprintf('poly%1.0f%1.0f',omegaPower,vPower);
f = fit( [ThrustVectors(:,1),ThrustVectors(:,2)],ThrustVectors(:,3), fittype);

figure (plotNum)
plotNum = plotNum + 1;
plot(f, [ThrustVectors(:,1),ThrustVectors(:,2)],ThrustVectors(:,3))
titlestring = sprintf('Thrust Produced Surface Fit\nUsed %s',fittype);
title(titlestring)
xlabel('Commanded Angular Velocity of the Propeller [rad/s]')
ylabel('Disk Climb Velocity [ft/s]')
zlabel('Thrust [lbsf]')

% Write Function
syms Omega vClimb Thrust x y
switch omegaPower
    case 2
        switch vPower
            case 2
                cv = coeffvalues(f);
                symsF = Thrust - formula(f);
                symsF = subs(symsF,x,Omega);
                symsF = subs(symsF,y,vClimb);
                eqans = solve(symsF,Omega);
                eqprint2 = sprintf('\nOmegaCMD = %s;\n\nend',char(eqans(2)));
                coeffprint = sprintf('p00 = %d;\np10 = %d;\np01 = %d;\np20 = %d;\np11 = %d;\np02 = %d;\n',cv(1),cv(2),cv(3),cv(4),cv(5),cv(6));
                
                %%% Print to File %%%
                fid = fopen('OmegaCalc.m','w');
                topstring = sprintf('function [OmegaCMD] = OmegaCalc(Thrust,vClimb)\n\n%%Calculate commanded motor angular velocity\n\n%%Coefficients:\n');
                fprintf(fid,topstring);
                fprintf(fid,coeffprint);
                fprintf(fid,eqprint2);
                fclose(fid);
                
                % The first solution is negative = not correct
                % eqprint1 = sprintf('\nOmega1 = %s;\n',char(eqans(1)));
                % fprintf(fid,eqprint1);
                % fprintf(fid,'\nOmegaCMD = [Omega1;Omega2];\n\nend');
                
        end
end


% Torque Curve Fit Parameters
omegaPower = 2;
vPower = 2;
fittype2 = sprintf('poly%1.0f%1.0f',omegaPower,vPower);
f = fit( [TorqueVectors(:,1),TorqueVectors(:,2)],TorqueVectors(:,3), fittype2);

figure (plotNum)
plotNum = plotNum + 1;
plot(f, [TorqueVectors(:,1),TorqueVectors(:,2)],TorqueVectors(:,3))
titlestring = sprintf('Torque Produced Surface Fit\nUsed %s',fittype2);
title(titlestring)
xlabel('Commanded Angular Velocity of the Propeller [rad/s]')
ylabel('Disk Climb Velocity [ft/s]')
zlabel('Torque [lbsf-ft]')

% Write Function
syms Omega vClimb Torque x y
switch omegaPower
    case 2
        switch vPower
            case 2
                cv = coeffvalues(f);
                symsF = formula(f);
                symsF = subs(symsF,x,Omega);
                symsF = subs(symsF,y,vClimb);
                eqans = symsF;
                eqprint2 = sprintf('\nTorqueOut = %s;\n\nend',char(eqans));
                coeffprint = sprintf('p00 = %d;\np10 = %d;\np01 = %d;\np20 = %d;\np11 = %d;\np02 = %d;\n',cv(1),cv(2),cv(3),cv(4),cv(5),cv(6));
                
                %%% Print to File %%%
                fid = fopen('TorqueCalc.m','w');
                topstring = sprintf('function [TorqueOut] = TorqueCalc(Omega,vClimb)\n\n%%Calculate commanded motor angular velocity\n\n%%Coefficients:\n');
                fprintf(fid,topstring);
                fprintf(fid,coeffprint);
                fprintf(fid,eqprint2);
                fclose(fid);
                
                % The first solution is negative = not correct
                % eqprint1 = sprintf('\nOmega1 = %s;\n',char(eqans(1)));
                % fprintf(fid,eqprint1);
                % fprintf(fid,'\nOmegaCMD = [Omega1;Omega2];\n\nend');
                
        end
end

end