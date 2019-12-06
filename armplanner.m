function[armplan, armplanlength,replanning_time,cost,first_planner_time,replanned,success] = armplanner(envmap,envmap_inflated, armstart, armgoal, planner_id)
%call the planner in C
size(envmap)
size(envmap_inflated)
[armplan, armplanlength,replanning_time,cost,first_planner_time,replanned,success] = run_planner(envmap, armstart, armgoal, planner_id,envmap_inflated);