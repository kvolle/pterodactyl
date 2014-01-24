A =[2.478633972466071  -0.208886656420609  -1.775940099696408;
   1.341344782030788  -0.782553466986132   1.228327471384109;
   0.096578915891449                   0  -0.136707496534875];
det(A)
%A = [1 0 0;0 1 0;0 0 1];
%
for i = 1:1000
    %q(i) =i;
    s(:,i) = A*[random('unif',0,2400)/100-12;random('unif',0,2400)/100-12;random('unif',0,2400)/100-13];
    %a(i)=s(2);
    
    %%plot3(s(1,i),s(2,i),s(3,i),'h');
    hold on
end
plot3(s(1,:),s(2,:),s(3,:),'k.');
%}
%

s = A*[12;-12;-12];
plot3(linspace(0,s(1),10),linspace(0,s(2),10),linspace(0,s(3),10),'rh');
hold on
plot3(linspace(0,s(1),10),linspace(0,s(2),10),linspace(0,s(3),10),'r');
s = A*[-12;12;12];
plot3(linspace(0,s(1),10),linspace(0,s(2),10),linspace(0,s(3),10),'rh');
plot3(linspace(0,s(1),10),linspace(0,s(2),10),linspace(0,s(3),10),'r');

s = A*[-12;12;-12];
plot3(linspace(0,s(1),10),linspace(0,s(2),10),linspace(0,s(3),10),'gh');
plot3(linspace(0,s(1),10),linspace(0,s(2),10),linspace(0,s(3),10),'g');
s = A*[12;-12;12];
plot3(linspace(0,s(1),10),linspace(0,s(2),10),linspace(0,s(3),10),'gh');
plot3(linspace(0,s(1),10),linspace(0,s(2),10),linspace(0,s(3),10),'g');

s = A*[-12;0;12];
plot3(linspace(0,s(1),10),linspace(0,s(2),10),linspace(0,s(3),10),'kh');
plot3(linspace(0,s(1),10),linspace(0,s(2),10),linspace(0,s(3),10),'b');
s = A*[12;0;-12];
plot3(linspace(0,s(1),10),linspace(0,s(2),10),linspace(0,s(3),10),'kh');
plot3(linspace(0,s(1),10),linspace(0,s(2),10),linspace(0,s(3),10),'b');
%}


xlabel('Roll')
ylabel('Pitch');
zlabel('Yaw');
view(2)
%t = abs(A)*[10;10;10];
%plot(q,t(2),'r');
%hold on
%plot(a)