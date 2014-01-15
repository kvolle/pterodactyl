clear all
clc

thrust = 12;
rho = 0.074757;

N = 2;
c = 0.125;
R  = .42;
sigma = N*c/(pi*R)

Cl_alpha = 2*pi
alpha = 3*pi/180
theta =3*pi/180
Cl = Cl_alpha*alpha;
Cd = 0.0108;

%{
for i =1:1000
    e = 25;
    omega(i) = i;
    for j = 1:e
        dCt = (1/e)*sigma*Cl*(j/e)^2;
    end
    T(i) = 0.5*rho*pi*(omega(i)^2)*(R^4);%*(1/261.267);

end
figure(1)
plot(omega,T,'c');

%}
%{
e = 250;
omega = zeros(1,30);
dT = zeros(1,e);
T = zeros(1,30);
dQ = zeros(1,3);
Q = zeros(1,30);
induced = sqrt(thrust/(2*rho*pi*R^2));
for i = 1:1000
    omega(i) = 1*i;
    for j = 1:e
        r = (j-0.5)*(R/e);
        %dT(j) = (R/e)*sqrt((omega(i)*r)^2 + induced^2)*(Cl*omega(i)*r-induced*Cd);
        %dT(j) = (R/e)*Cl_alpha*(theta-$$$/omega(i)*r)(omega(i)*r)^2
        dT(j) = (R/e)*(omega(i)*r)*(Cl*omega(i)*r);
        %dQ(j) = (R/e)*sqrt((omega(i)*r)^2 + induced^2)*(Cd*omega(i)*r^2+induced*r*Cl);
    end
    T(i) = N*rho*c*sum(dT);
    %Q(i) = N*rho*c*sum(dQ)/2;
    %ratio(i) = Q(i)/T(i);
end



plot(omega,T,'r');

%plot(omega,Q,'b');
%}

%
lambda = -sigma*Cl_alpha/16 + 0.5*sqrt((sigma*Cl_alpha/8)^2 + sigma*Cl_alpha*theta/3)

Ct = sigma*Cl_alpha*(theta/6 - lambda/4);
Cq  = sigma*Cd/8 + sigma*lambda*Cl_alpha*(theta/6 - lambda/4);
if abs(lambda-sqrt(Ct/2))>0.0000001
    disp('HELP')
end
for i = 1:1000
    omega(i) = i;
    T2(i) =Ct*rho*pi*(omega(i)^2)*(R^4);
    Q(i) = Cq*rho*pi*(omega(i)^2)*(R^5);
    ratio(i) = Q(i)/T2(i);
end
figure(2)
plot(omega,T2,'g');
%{
figure(1)
plot(omega,T2,'g');
figure(2)
plot(omega,Q,'g');
figure(3)
plot(omega,ratio,'r');
%}
mean(ratio)
%Omega = sqrt(thrust/(Ct*rho*pi*R^4))
omega_induced = sqrt(thrust/(2*rho*pi*R^2));
% Approximate hover so assume V = 0 for now
%lambda*Omega*R
%}
