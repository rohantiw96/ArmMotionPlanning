format short

num_of_samples = 5;
armplanlength_ = zeros(1,num_of_samples);
replanning_time_ = zeros(1,num_of_samples);
cost_ = zeros(1,num_of_samples);
first_planner_time_ = zeros(1,num_of_samples);
success_ = zeros(1,num_of_samples);

counter = 1;

for i=1:num_of_samples
    mex run_planner.cpp sampling_planner.cpp -I./include -I./LazyPRM LazyPRM/lazy_prm.cpp -I./DRRT DRRT/DRRT.cpp -I/usr/local/include
    [armplanlength,replanning_time,cost,first_planner_time,replanned,success] = runtest('Maps/test2.mat','Maps/test2_infl.mat',[7*pi/8 pi pi pi pi/2],[pi/8 0 0 0 pi/2],0);
    armplanlength_(i) =armplanlength;
    replanning_time_(i) = replanning_time;
    cost_(i) = cost;
    first_planner_time_(i) = first_planner_time;
    success_(i) = success;
end
armplanlength_ave = mean(armplanlength_);
replanning_time_ave = mean(replanning_time_);
cost_ave = mean(cost_);
first_planner_time_ave = mean(first_planner_time_);
succes_ave = mean(success_);