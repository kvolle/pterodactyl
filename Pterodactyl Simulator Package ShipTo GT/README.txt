This README document provides a summary of the existing matlab functions written by Trevor Bennett as documented by Kyle Volle

29 May 2013

AerodynamicProperties.m
    This function takes no arguments
    This function uses matlab's stopwatch feature
    First defines the air properties, temperature in degrees F and pressure in psf (I believe)
    Next defines lift and drag coefficients
    After that defines Moment coefficients, in particular the moment about the Quarter Chord
    Next the data is rotated from the wind frame to the airfoil fixed frame
    Re-mesh the data (I have a lot to learn)
%%%%%% Coming back to this function later %%%%%%

BEMT_ControlCoeffs
    BEMT Input Variables: Motor Rotation Rate, Velocity of Hub
    BEMT Output Variables: Thrust
    %%%%%% Either the above or the next line is wrong as written in the function comments. Will correct one when finished with this function
    This function generates an equation that takes thrust and velocity as input aguments and returns a commanded motor rotation rate
    Function defines altitude on line 21 in a way that I am not familiar with
    Define the extreme neg/pos free stream velocities and the velocity into the hub
    Define stepsize to be 0.5 (units?)
    valslow is a vector of linearly spaced points between lowest hub velocity and zero. The number of points is given by minval/stepsize (rounded up)
    valshigh is a vector of linearly spaced points from stepsize to maximum hub velocity. The number of points is given by (maxval/stepsize)-1 (rounded up).
    Not sure what vInfSet is
    nCurves is the length of vInfSet

    Set Commanded Motor Rotation Rate in rad/s
    maxRPM = 12000
    minRPM = 3000
    RPM increments in units of 100 rpm
    rotRateCMDSet is a vector of linearly spaced points between min rad/s and max rad/s with points stread out by 100 rpm increments
    nCMDs is the length of the rotRateCMDSet

    ThrustMatrix is a nCMDs by nCurves matrix
    ThrustVectors is a 3 column matrix with one row for every element in ThrustMatrix
    TorqueMatrix is the same size as ThrustMatrix
    TorqueVectors is the same size as ThrustVectors
    b_ind holds the value of the length of global variable Bladex (which doesn't appear to have been set)
    Nested loops, x insdex from 1 to nCMDs and y index from 1 to nCurves
    Each loop through, Omega is equal to the value in the first0 column of that row of rotRateCMDSet
    Inside the second loop vInf is taken from the yind (y index) row of vInfSet, column 1
    Vclimb is set equal to VInf
    % Assuming constant Vclimb for all annuli of a propeller: lambdac = Vclimb/(Omega*BladeR)
    %%%%%% I don't know where Bladex, BladeR et al are set %%%%%%
    zero out integration indices and temp
    For each element in Bladex
      x = Bladex
      r = BladeR*x
      c = BladeChord
      Cla = BladeCLAl
      Cd0 = BladeCD0
      theta = BladePitch
      sigma = BladeN*c/(pi*BladeR)
      lambda? = -(sigma*Cla/16 - lambdac/2) + sqrt((sigma*Cla/16 - lambdac/2)^2 + sigma*Cla*theta*x/8)
      %%%%%% What are sigma and lambda? %%%%%%
      induced velocity w = Omega(BladeR*(lambda-lambdac)
      Numerical integration for T and Q
      %%%%%% What are T and Q %%%%%%
      vterm = Vclimb + w
      rterm = Omega*r
      Cl = Cla*(theta-vter/rterm) + BladeCL0

      T2(1,1) = x (Bladex)
      T2(1,2) = c*rterm*(rterm*Cl - vterm * Cd0)
      Ttemp = Ttemp + BladeR*(x-T1(1,1))*(T1(1,2)+T2(1,2))/2
      T1 = T2
      %%%%%% Check this out later, there has to be a better way of doing this. Repeated for Q
      % Compute integral and store
      Thrust and torque curve fitting code
      %%%%%% Understand this later %%%%%%