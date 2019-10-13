function collision_value = EvaluatePointWithinPolygonDegree(P, V)

global kLargeVal

len = size(V, 1);
areaV = CalculateArea(V);
V = [V; V(1,:)];

s_gross = 0;
for ii = 1 : len
    Vtriangle = [V(ii,:); V(ii+1,:); P];
    s_gross = s_gross + CalculateArea(Vtriangle);
end

if (s_gross > areaV + 0.00001)
    collision_value = 0;
else
    collision_value = kLargeVal;
end