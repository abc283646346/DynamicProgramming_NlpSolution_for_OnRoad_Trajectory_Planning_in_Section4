function [x, y, theta] = ConvertIndexToTrajectory(result_index)
global init_s init_l ds min_l dl BV
x = init_s;
y = init_l;
for ii = 2 : size(result_index,1)
    x = [x, init_s + ds * (ii-1)];
    offset_index = result_index(ii,2);
    y = [y, min_l + dl * (offset_index-1)];
end
theta = zeros(1, length(x));
theta(1,1) = BV.theta0;
for ii = 2 : length(x)
    theta(ii) = atan2(y(ii) - y(ii -1), x(ii) - x(ii-1));
end