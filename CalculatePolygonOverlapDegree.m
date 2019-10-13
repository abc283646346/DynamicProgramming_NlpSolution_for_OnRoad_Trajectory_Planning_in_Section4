function [collision_occur_flag, degree] = CalculatePolygonOverlapDegree(vx, vy, Vcar)
global kObstacleRepulsiveDistance
collision_occur_flag = 0;
nvp = length(vx) - 1;
degree = exp(-kObstacleRepulsiveDistance * hypot(mean(vx(1:nvp)) - mean(Vcar.x), mean(vy(1:nvp)) - mean(Vcar.y)));

if (sum(inpolygon(Vcar.x, Vcar.y, vx, vy)))
    collision_occur_flag = 1;
    return;
end

if (sum(inpolygon(vx, vy, [Vcar.x, Vcar.x(1)], [Vcar.y, Vcar.y(1)])))
    collision_occur_flag = 1;
    return;
end
end