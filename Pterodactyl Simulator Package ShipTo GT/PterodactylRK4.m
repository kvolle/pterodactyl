function [ynplus] = PterodactylRK4(t_step, yn, CMD)
% Fourth Order Runge-Kutta
%   This Function is specifically designed for the Pterodactyl 6-DOF Flight
%   Simulator and utilizes the PterodactylKinetics function
%     Note: The traditional Runge Kutta passes in both the current time
%     plus some variation in time given the coefficient in addition to
%     passing in the current state plus some variation in the state. This
%     function only passes in the variation in time instead of the current
%     time plus variation. The pass in of the state and varation to the
%     state is consistent with the Runge Kutta format.

% Change Log
%{
    9/2012 - Function Written by Trevor Bennett
    5/15/2013 - Code Packaged by Trevor Bennett
%}

% Calculate Coefficients
k1 = t_step*PterodactylKinetics(0,yn,CMD);

yn2 = state_qcheck(yn + 0.5*k1);
k2 = t_step*PterodactylKinetics(0.5*t_step,yn2,CMD);

yn3 = state_qcheck(yn + 0.5*k2);
k3 = t_step*PterodactylKinetics(0.5*t_step,yn3,CMD);

yn4 = state_qcheck(yn + k3);
k4 = t_step*PterodactylKinetics(t_step,yn4,CMD);

% Compile y(n+1)
ynplus = state_qcheck(yn + (k1 + 2*k2 + 2*k3 +k4)/6);
end

