function val = CalculateCurvatureCost(p1, p2, p3)
global max_curvature_half_cost
val = abs(atan2(p2(2)-p1(2), p2(1)-p1(1)) - atan2(p3(2)-p2(2), p3(1)-p2(1))) / (max_curvature_half_cost * 2);
end