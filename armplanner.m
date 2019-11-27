function[armplan, armplanlength,time,cost,vertices] = armplanner(envmap, armstart, armgoal, planner_id)
%call the planner in C
[armplan, armplanlength,time,cost,vertices] = run_planner(envmap, armstart, armgoal, planner_id);