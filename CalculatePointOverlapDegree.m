function [collision_occur_flag, degree] = CalculatePointOverlapDegree(x, y, V)
global kObstacleRepulsiveDistance
if (inpolygon(x, y, [V.x, V.x(1)], [V.y, V.y(1)]))
    collision_occur_flag = 1;
else
    collision_occur_flag = 0;
end
degree = exp(-kObstacleRepulsiveDistance * hypot(mean(V.x(1:4)) - x, mean(V.y(1:4)) - y));
end