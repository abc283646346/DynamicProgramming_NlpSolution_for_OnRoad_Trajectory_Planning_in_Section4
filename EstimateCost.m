function [val, time_next_layer_state] = EstimateCost(cur_state_ind, next_layer_state_ind)
global state_space
global min_s min_l min_v
global init_s init_l init_v
global ds dl dv
if (cur_state_ind(1) == 0)
    cur_s = init_s;
    cur_l = init_l;
    cur_v = init_v;
else
    cur_s = min_s + ds * cur_state_ind(1);
    cur_l = min_l + dl * (cur_state_ind(2) - 1);
    cur_v = min_v + dv * (cur_state_ind(3) - 1);
end
next_s = min_s + ds * next_layer_state_ind(1);
next_l = min_l + dl * (next_layer_state_ind(2) - 1);
next_v = min_v + dv * (next_layer_state_ind(3) - 1);

if (cur_state_ind(1) == 0)
    current_moment = 0;
    current_cost = 0;
else
    current_moment = state_space{cur_state_ind(1), cur_state_ind(2), cur_state_ind(3)}(5);
    current_cost = state_space{cur_state_ind(1), cur_state_ind(2), cur_state_ind(3)}(1);
end
time_next_layer_state = current_moment + ds / (0.5 * (cur_v + next_v));
time_interval = [current_moment, time_next_layer_state];
s_interval = [cur_s, next_s];
l_interval = [cur_l, next_l];

cost_from_current_to_next = CalculateCostFromCurrentToNext(cur_state_ind, next_layer_state_ind, time_interval, s_interval, l_interval);
val = current_cost + cost_from_current_to_next;
end
