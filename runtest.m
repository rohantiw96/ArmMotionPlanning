function[armplanlength,replanning_time,cost,first_planner_time,replanned,success] = runtest(mapfile, mapfile_inflated, armstart, armgoal, planner_id)

map = load(mapfile);
map_inflated = load(mapfile_inflated);
close all;

%armplan should be a matrix of D by N 
%where D is the number of DOFs in the arm (length of armstart) and
%N is the number of steps in the plan 
mapT = permute(map.map,[2 1 3]);
map_inflatedT = permute(map_inflated.map, [2 1 3]);
<<<<<<< HEAD
[armplan, armplanlength,replanning_time,cost,first_planner_time,replanned,success] = armplanner(mapT,map_inflatedT, armstart, armgoal, planner_id); 
fprintf(1, 'Arm trajectory of %d waypoints was returned\n', size(armplan,1));
=======
[armplan, armplanlength,time,cost,vertices] = armplanner(mapT,map_inflatedT, armstart, armgoal, planner_id); 
fprintf(1, 'Arm trajectory of %d  was returned\n', size(armplan,1));

>>>>>>> debugDRRT
%Animate
animate(map.map, armplan, size(armplan,1));
end