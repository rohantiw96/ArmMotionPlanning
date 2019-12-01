function[armplanlength,time,cost,vertices] = runtest(mapfile, armstart, armgoal, planner_id)

envmap = load(mapfile);
close all;

%armplan should be a matrix of D by N 
%where D is the number of DOFs in the arm (length of armstart) and
%N is the number of steps in the plan 
[armplan, armplanlength,time,cost,vertices] = armplanner(envmap, armstart, armgoal, planner_id); 
fprintf(1, 'plan of length %d was found\n', size(armplan,1));

%Animate
animate(envmap, armplan, armplanlength);

end