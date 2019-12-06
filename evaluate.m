format short

num_of_samples = 5;
armplanlength_ = zeros(1,num_of_samples);
replanning_time_ = zeros(1,num_of_samples);
cost_ = zeros(1,num_of_samples);
first_planner_time_ = zeros(1,num_of_samples);
success_ = zeros(1,num_of_samples);

counter = 1;

for i=1:num_of_samples
    mex run_planner.cpp sampling_planner.cpp -I./include -I./LazyPRM LazyPRM/lazy_prm.cpp -I./DRRT DRRT/DRRT.cpp
    [armplanlength,replanning_time,cost,first_planner_time,replanned,success] = runtest('Maps/map6.mat','Maps/map6_infl.mat',[pi/8 3*pi/4 pi 0.9*pi 1.5*pi],[pi/2 pi/2 pi/2 pi/4 pi/2],0);

    armplanlength_(i) =armplanlength;
    replanning_time_(i) = replanning_time;
    cost_(i) = cost;
    first_planner_time_(i) = first_planner_time;
    success_(i) = success;
end
armplanlength_ave = mean(armplanlength_)
replanning_time_ave = mean(replanning_time_)
cost_ave = mean(cost_)
first_planner_time_ave = mean(first_planner_time_)
succes_ave = mean(success_)