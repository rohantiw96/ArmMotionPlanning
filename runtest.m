function[armplanlength,time,cost,vertices] = runtest(mapfile,mapfile_inflated, armstart, armgoal, planner_id)

load(mapfile, 'map');
load(mapfile_inflated, 'map_inflated');

close all;

%armplan should be a matrix of D by N 
%where D is the number of DOFs in the arm (length of armstart) and
%N is the number of steps in the plan 
[armplan, armplanlength,time,cost,vertices] = armplanner(map, armstart, armgoal, planner_id,map_inflated); 
fprintf(1, 'plan of length %d was found\n', size(armplan,1));

%Animate
animate(map, armplan, size(armplan,1));

end