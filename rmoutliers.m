function [cleaned_array] = rmoutliers(input,idx)

mean_input = mean(input,idx,'omitnan');
std_input = std(input,0,idx,'omitnan');

clean = (input < mean_input + 3*std_input) & (input > mean_input - 3*std_input);

cleaned_array = input.*clean;


end

