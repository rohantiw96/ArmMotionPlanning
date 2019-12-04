function[armplan, armplanlength,time,cost,vertices] = armplanner(envmap,envmap_inflated, armstart, armgoal, planner_id)
%call the planner in C
size(envmap)
size(envmap_inflated)
[armplan, armplanlength,time,cost,vertices] = run_planner(envmap, armstart, armgoal, planner_id,envmap_inflated);