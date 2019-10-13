clear all
close all
clc

global x_horizon y_horizon_ub y_horizon_lb
x_horizon = 120;
y_horizon_ub = 3.75;
y_horizon_lb = -3.75;

global static_grids
num_grids = 121;
static_grids.x = linspace(0, x_horizon, num_grids);
static_grids.y_ub = y_horizon_ub + 0.3 * rand(1,num_grids) + randn(1,num_grids) * 0.01;
static_grids.y_lb = y_horizon_lb - 0.3 * rand(1,num_grids) - randn(1,num_grids) * 0.01;
global dynamic_obstacles full_dynamic_obstacles Nfe maximum_time_horizon
Nfe = 101;
num_obstacles = 8;
maximum_time_horizon = 10;
num_grids = 31;
abstracted_index_set = round(linspace(1, Nfe, num_grids));
full_timeline = linspace(0, maximum_time_horizon, Nfe);
timeline = linspace(0, maximum_time_horizon, num_grids);
dynamic_obstacles = cell(num_obstacles, num_grids);
full_dynamic_obstacles = cell(num_obstacles, Nfe);
for ii = 1 : num_obstacles
    origin_x = rand * (x_horizon - 20) + 20;
    origin_y = rand * 6 - 3;
    basic_vertex_x_pattern = [-2 2 2 -2 -2];
    basic_vertex_y_pattern = [0.75 0.75 -0.75 -0.75 0.75];
    dx = linspace(0, 20 + 50 * rand, Nfe);
    dy = cos(0.1 * [1 : Nfe] + rand * 0.1) * 0.2;
    for jj = 1 : Nfe
        elem.time = full_timeline(jj);
        elem.x = origin_x + basic_vertex_x_pattern + dx(jj);
        elem.y = origin_y + basic_vertex_y_pattern + dy(jj);
        full_dynamic_obstacles{ii, jj} = elem;
    end
end

for ii = 1 : num_obstacles
    for jj = 1 : num_grids
        index = abstracted_index_set(jj);
        dynamic_obstacles{ii, jj} = full_dynamic_obstacles{ii, index};
    end
end

global BV
BV.x0 = 0;
BV.y0 = -1;
BV.theta0 = rand * 0.1;
BV.v0 = 15;
BV.a0 = rand * 0.01;
BV.phy0 = 0;
BV.w0 = 0;

global coarse_x coarse_y coarse_theta
InitParams;
[x, y, theta, time] = MakeTrajectoryDecisionViaDp();
[coarse_x, coarse_y, coarse_theta] = AdjustProfileTimeline(x, y, theta, time, timeline);
fixed_tf = maximum_time_horizon;
DomonstrateCoarseTrajectory();

[coarse_x, coarse_y, coarse_theta, ~] = ResampleConfig(coarse_x, coarse_y, coarse_theta, coarse_theta, Nfe);
WriteFilesForNLP(fixed_tf);
WriteInitialGuessForNLP(coarse_x, coarse_y, coarse_theta, fixed_tf);

!ampl r2.run
!ampl r3.run

dsa();
% asd();
