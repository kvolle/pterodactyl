classdef quatRK4
    methods
        function obj = quatRK4(State, Ib, m, Force, Moment);
            obj.State = State;
            obj.Ib = Ib;
            obj.m = m;
            obj.Force = Force;
            obj.Moment = Moment;
        end
        function dState = stateDiff(obj, y)
            % THIS DOES NOT INCLUDE GRAVITY
            Po = y(1:3);
            Qu = y(4:7);
            Ve = y(8:10);
            An = y(11:13);
            
            % Define the three matrices used in calculating the diff eqs
            TransKinDiffEq = [1-2*(Qu(3)^2+Qu(4)^2) 2*(Qu(2)*Qu(3)-Qu(1)*Qu(4)) 2*(Qu(2)*Qu(4)+Qu(1)*Qu(3));
                              2*(Qu(2)*Qu(3)+Qu(1)*Qu(4)) 1-2*(Qu(4)^2+Qu(2)^2) 2*(Qu(3)*Qu(4)-Qu(1)*Qu(2));
                              2*(Qu(2)*Qu(4)-Qu(1)*Qu(3)) 2*(Qu(3)*Qu(4)+Qu(1)*Qu(2)) 1-2*(Qu(2)^2+Qu(3)^2)];
              
            RotKinDiffEq = [-Qu(2) -Qu(3) -Qu(4);Qu(1) -Qu(4) Qu(3); Qu(4) Qu(1) -Qu(2);-Qu(3) Qu(2) Qu(1)];

            DyDiffEq = [0, -An(3), An(2);An(3),0, -An(1);-An(2), An(1), 0];
            % Calculate Differential Equations
            dPo = TransKinDiffEq*Ve;
            dQu = RotKinDiffEq*An;
            dVe = (1/obj.m)*obj.Force - DyDiffEq*Ve;
            tmp = obj.Moment - DyDiffEq*obj.Ib*An;
            dAn = obj.Ib\tmp;

            dState(1:3,1) = dPo;
            dState(4:7,1) = dQu;
            dState(8:10,1) = dVe;
            dState(11:13,1) = dAn;
        end
        function newState = homebrewRK4(obj)
            dt = 0.002;
            k1 = obj.stateDiff(obj.State);
            k2 = obj.stateDiff(obj.State+(dt/2)*k1);
            k3 = obj.stateDiff(obj.State+(dt/2)*k2);
            k4 = obj.stateDiff(obj.State+dt*k3);
            newState = obj.State + (dt/6)*(k1+2*k2+2*k3+k4);
        end
        
    end
    
    properties
            State
            Ib
            m
            Force
            Moment
    end
 end
    