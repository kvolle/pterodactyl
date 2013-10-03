function[htm,thrustHTM,velocity,alpha,lift,drag,pitchMoment,area]= AerodynamicCoefficients(state)
[htm,area,thrustHTM] = PterodactylGeometry;
[~, Alpha, ClAlphain, CdAlphain, AlphaCm, ReCm,CmAlpha] = AerodynamicProperties;

% Sea level, 15 deg C
rho = 0.076474; % lbm/ft^3
% Ask Dr Rogers about Reynolds Number to use
Re = 160000; % This is just a place holder, it isn't used
             % Will end up needing to do 2D interpolation
% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Calculate the velocity and angle of attack (alpha) of each mean aerodynamic centers
  % Malloc
  velocity = zeros(3,44);
  alpha = zeros(1,44);
  lift = zeros(1,44);
  drag = zeros(1,44);
  pitchMoment = zeros(1,44);
 

  for i = 1:44
    velocity(:,i) = state(8:10) + [0 -state(13) state(12);state(13) 0 -state(11);-state(12) state(11) 0]*htm(1:3,4,i);
    alpha(1,i) = atan2(velocity(3,i),velocity(1,i));
    j =1;
    % Next 5 lines interpolate lift coefficient and drag coefficient
    % TODO calculate Re and interpolate between columns
    
    if (alpha(i) < 0)
        alpha(i) = alpha(i) + 2*pi;
    end
    
    while(Alpha(j+1) < alpha(i)*180/pi)
        j=j+1;
    end
    cl = ClAlphain(j) + (ClAlphain(j+1)-ClAlphain(j))*(alpha(i)*180/pi-Alpha(j))/(Alpha(j+1)-Alpha(j));
    cd = CdAlphain(j) + (CdAlphain(j+1)-CdAlphain(j))*(alpha(i)*180/pi-Alpha(j))/(Alpha(j+1)-Alpha(j));

    % NOTE This isn't true airspeed and doesn't account for wind
    lift(i) = rho*area(i)*(velocity(1,i)^2 + velocity(2,i)^2 + velocity(3,i)^2)*cl/2;
    %%test(i) = rho*area(i)*(velocity(1,i)^2 + velocity(2,i)^2 + velocity(3,i)^2)/2;
    drag(i) = rho*area(i)*(velocity(1,i)^2 + velocity(2,i)^2 + velocity(3,i)^2)*cd/2;
    
    j = 1;
    while(AlphaCm(j+1) < alpha(i)*180/pi)
        j = j+1;
    end
    cm = CmAlpha(j) + (CmAlpha(j+1)-CmAlpha(j))*(alpha(i)*180/pi-AlphaCm(j))/(AlphaCm(j+1)-AlphaCm(j));
    pitchMoment(i) = rho*area(i)*(velocity(1,i)^2 + velocity(2,i)^2 + velocity(3,i)^2)*cm/2;
 
  end