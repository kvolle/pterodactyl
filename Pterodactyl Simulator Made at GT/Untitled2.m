n = null(A(1:3,:));
bodyDir = A(4:6,:)*n;
bodyDir = bodyDir/norm(bodyDir);
for i = 1:10
     d = dot(bodyDir,forceDir);
    theta = acos(d);
    c = cross(torqueFree,forceDir);
    c = sin(theta/2)*c/norm(c);

    w = cos(theta/2);
    x = c(1);
    y = c(2);
    z = c(3);

    w2 = w^2;
    x2 = x^2;
    y2 = y^2;
    z2 = z^2;

    Rd = [w2+x2-y2-z2 2*(x*y-w*z) 2*(w*y+x*z);2*(x*y+w*z) w2-x2+y2-z2 2*(y*z-w*x);2*(x*z-w*y) 2*(w*x+y*z) w2-x2-y2+z2];
    setYaw = atan2(Rd(3,2),Rd(3,3));
    setPitch = atan2(-Rd(3,1),Rd(3,3)/cos(setYaw));
    setRoll = atan2(Rd(2,1),Rd(1,1));
    
        x = [copter.State(4)-setRoll;copter.State(10);copter.State(5)-setPitch;copter.State(11);copter.State(6)-setYaw;copter.State(12)];
    f = a*x;
    
    % Calculate the moment to command in world coordinates
    worldMom = Ib*[f(2);f(4);f(6)];
    bodyMom = R\worldMom;
    %bodyMom = [0;0;0];

    thrust = [A(1:3,:);A(6,:)]\[bodyMom;forceMag*forceDir(3)];
 
     %thrust = A2\[bodyMom(1);bodyMom(2)];
     
     if max(abs(thrust))>15
         thrust = 15*thrust/max(abs(thrust));
     end
     
     bodyDir = A(4:6,:)*thrust;
     bodyDir = bodyDir/norm(bodyDir);
end
    bodyDir