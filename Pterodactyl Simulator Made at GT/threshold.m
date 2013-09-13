function vector = threshold(vector);
for i = 1:size(vector)
    if (abs(vector(i)) < 10^-8)
        vector(i) = 0;
    end
end