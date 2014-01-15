clear all
clc
Ix = zeros(15,15,5);
Iy = zeros(15,15,5);
Iz = zeros(15,15,5);
for i = 1:15
    for j = 1:15
        for k = 1:5
            [tmp1,tmp2,tmp3,Ix(i,j,k),Iy(i,j,k),Iz(i,j,k)] = massCenter(15,5*i+65,5*j+70,3*k);
            X(i) = 5*i+65;
            Y(j) = 5*j+70;
            total(i,j,k) = norm([Ix(i,j,k),Iy(i,j,k),Iz(i,j,k)]);
        end
    end
end
surf(X,Y,total(:,:,4));