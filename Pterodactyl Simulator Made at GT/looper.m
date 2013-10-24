 clear all
 clc
 tic
%count = zeros(5,5,20);
count1 = zeros(250,1);
count2 = zeros(250,1);
count3 = zeros(250,1);
count4 = zeros(250,1);
err1 = zeros(250,1);
err2 = zeros(250,1);
err3 = zeros(250,1);
err4 = zeros(25,1);
running = 0;
        for i = 1:250
            [count4(i),err4(i)] = test(2.5,3,7.5);
            running = running+count4(i);
        end



sum(count4)

toc
