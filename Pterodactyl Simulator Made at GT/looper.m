 clear all
 clc
 tic
count = zeros(10,1);
err = zeros(10,1);

meanErr = zeros(10,10,6);
stableTable = zeros(10,10,6);
maxTable = zeros(10,10,6);
for i = 1:10
    for j = i+1:10
        for k = 1:6;
            for x = 1:10
                %[count4(i),err4(i)] = test(2.5,3,7.5);
                [count(x),err(x)] = test(i*10,j*10,k*10);
                %running = running+count4(i);
            end
            stableTable(i,j,k) = sum(count);
            maxTable(i,j,k) = max(err);
            meanErr(i,j,k) = mean(err);
            10*(i-1)+j
        end
    end
end



sum(count4)
plot(err4)
toc
