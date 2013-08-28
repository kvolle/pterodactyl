function dState = stateDiff(state,Ib,m,Force,Moment)
%Ib = [1 0 0;0 1 0; 0 0 1];
%m = 2;

% Initial states
% Sphere of 2 kg 
Po = state(1:3);%[0;0;0];
Or = state(4:6);%[0;0;0];
Ve = state(7:9);%[0;0;0];
An = state(10:12);%[0;0;0];

% FOrces and Moments in body frame

%Force = [2;0;0];
%Moment = [0;0;0];

% Define the three matrices used in calculating the diff eqs
TransKinDiffEq = [cos(Or(2))*cos(Or(3)), sin(Or(1))*sin(Or(2))*cos(Or(3))-cos(Or(1))*sin(Or(3)), cos(Or(1))*sin(Or(2))*cos(Or(3))-sin(Or(1))*sin(Or(3));
                  cos(Or(2))*sin(Or(3)), sin(Or(1))*sin(Or(2))*sin(Or(3))-cos(Or(1))*cos(Or(3)), cos(Or(1))*sin(Or(2))*sin(Or(3))-sin(Or(1))*cos(Or(3));
                  -sin(Or(2)), sin(Or(1))*cos(Or(2)), cos(Or(1))*cos(Or(2))];
              
RotKinDiffEq = [1, sin(Or(1))*tan(Or(2)), cos(Or(1))*tan(Or(2)); 0, cos(Or(1)), -sin(Or(1)); 0, sin(Or(1))/cos(Or(2)), cos(Or(1))/cos(Or(2))];

DyDiffEq = [0, -An(3), An(2);An(3),0, -An(1);-An(2), An(1), 0];

% Calculate Differential Equations
dPo = TransKinDiffEq*Ve;
dOr = RotKinDiffEq*An;
dVe = (1/m)*Force - DyDiffEq*Ve;
tmp = Moment - DyDiffEq*Ib*An;
dAn = Ib\tmp;

dState(1:3) = dPo;
dState(4:6) = dOr;
dState(7:9) = dVe;
dState(10:12) = dAn;
%%%%%%%%%%%%%%%%%%%%%%% Major issues with variable flow

