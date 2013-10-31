clear all
clc

A = rand(100,100,6);
[x,y,z] = meshgrid(1:100,1:100,1:6);
scatter3(x(:),y(:),z(:),5,A(:))