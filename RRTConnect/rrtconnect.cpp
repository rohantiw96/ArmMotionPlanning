#include"rrtconnect.h"

RRTConnect::RRTConnect(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int sampling_rate,double bias_probability,int max_iterations)
        :SamplingPlanners(map,x_size,y_size,arm_start,arm_goal,numofDOFs){
        epsilon_ = epsilon;
        sampling_rate_ = sampling_rate;
        bias_generator_ = std::mt19937(std::random_device()());
        distribution_goal_selection_ = std::uniform_int_distribution<int>(1, 100);
        bias_probability_ = bias_probability;
        max_iterations_ = max_iterations;
}

void RRTConnect::addNode(const std::vector<double> &q_near,const std::vector<double> &q_new,bool is_goal){
    if (is_goal) {
        tree_goal_[q_new] = q_near;
    }
    else {
        tree_[q_new] = q_near;
    }
}

std::vector<double> RRTConnect::findNearestNeighbor(const std::vector<double> &q_rand,const bool is_goal){
    std::vector<double> nearest_neighbor;
    double min_distance = INT_MAX;
    double euclidean_distance = 0;
    if (is_goal){
        for(const auto& n:tree_goal_){
            euclidean_distance = euclideanDistance(n.second,q_rand);
            if(euclidean_distance < min_distance){
                min_distance = euclidean_distance;
                nearest_neighbor = n.second;
            }
        }
    }
    else{
        for(const auto& n:tree_){
            euclidean_distance = euclideanDistance(n.first,q_rand);
            if(euclidean_distance < min_distance){
                min_distance = euclidean_distance;
                nearest_neighbor = n.first;
            }
        }
    }
    return nearest_neighbor;
}
std::vector<double> RRTConnect::biasedAngleSampling(const double bias_probability,const std::vector<double>& node){
  std::vector<double> angles;
  if (distribution_goal_selection_(bias_generator_) > bias_probability*100){
    for(int i=0;i<numofDOFs_;i++){

      angles.push_back(angle_distribution_(generator_));
    }
  }
  else{
    angles = node;
  }
  return angles;
}

std::vector<double> RRTConnect::interpolateBetweenNodes(const std::vector<double>& start,const std::vector<double>& end){
    std::vector<double> delta;
    std::vector<double> collision_free_configeration;
    std::vector<double> q_current = start;
    for(int i=0;i<numofDOFs_;i++){
        delta.push_back((end[i] - start[i])/num_samples_);
    }
    for(int i=0; i < num_samples_; i++){
        collision_free_configeration = q_current;
        for(int j=0;j<numofDOFs_;j++){
            q_current[j] = q_current[j] + delta[j];
        }
        wrapAngles(q_current);
        if (!IsValidArmConfiguration(q_current,true)){
            return collision_free_configeration;
        }
    }
    return end;
}

std::vector<double> RRTConnect::extend(const std::vector<double> &q_start,const std::vector<double> &q_end){
    std::vector<double> q_epsilon;
    for(int i = 0;i <numofDOFs_;i++){
        q_epsilon.push_back(q_end[i] - q_start[i]);
    }
    double norm = getNorm(q_epsilon);
    if (norm <= epsilon_) return q_end;
    for(int i=0;i<numofDOFs_;i++){
        q_epsilon[i] = q_start[i] + q_epsilon[i]/norm * epsilon_;
    }
    return q_epsilon;
}

std::vector<double> RRTConnect::extendNode(std::vector<double> q_new,const bool is_goal){
    std::vector<double> q_near= findNearestNeighbor(q_new,is_goal);
    std::vector<double> q_epilison = extend(q_near,q_new);
    std::vector<double> collision_free_configeration = interpolateBetweenNodes(q_near,q_epilison);
    if (collision_free_configeration != q_near){
        addNode(q_near,collision_free_configeration,is_goal);
        return collision_free_configeration;
    }
    return q_near;   
}

std::vector<double> RRTConnect::joinNode(std::vector<double> q_exteded,const bool is_goal){
    std::vector<double> q_near= findNearestNeighbor(q_exteded,is_goal);
    std::vector<double> collision_free_configeration = interpolateBetweenNodes(q_near,q_exteded);
    if (collision_free_configeration != q_near){
        addNode(q_near,collision_free_configeration,is_goal);
        return collision_free_configeration;
    }
    return q_near;
}

std::vector<std::vector<double> > RRTConnect::getPath(const std::vector<double> &angles){
    std::vector<std::vector<double> > path;
    std::vector<double> q_current = angles;
    while(tree_[q_current] != arm_start_){
        path.push_back(q_current);
        q_current = tree_[q_current];
    }
    path.push_back(arm_start_);
    std::reverse(std::begin(path), std::end(path));
    return path;
}

std::vector<std::vector<double> > RRTConnect::getPathToGoal(const std::vector<double> &angles){
    std::vector<std::vector<double> > path;
    std::vector<double> q_current = angles;
    while(tree_goal_[q_current] != arm_goal_){
        path.push_back(q_current);
        q_current = tree_goal_[q_current];
    }
    path.push_back(arm_goal_);
    return path;
}

void RRTConnect::getFirstPlan(std::vector<std::vector<double>> &plan){
    bool reachedGoal = false;
    tree_[arm_start_] = std::vector<double>{};
    tree_goal_[arm_goal_] = arm_goal_;
    std::vector<double> q_new;
    std::vector<double> collision_free_configeration;
    std::vector<double> collision_free_configeration_other;
    std::vector<std::vector<double>> path = std::vector<std::vector<double>>{};
    std::vector<std::vector<double>> path_to_goal = std::vector<std::vector<double>>{};
    bool is_goal = false;
    int j = 0;
    if (!checkGoalAndStartForCollision()){
        while(!reachedGoal){
            if (is_goal){
                q_new = biasedAngleSampling(bias_probability_,arm_start_);
            }
            else{
                q_new = biasedAngleSampling(bias_probability_,arm_goal_);
            }
            collision_free_configeration = extendNode(q_new,is_goal);
            if (!collision_free_configeration.empty()){
                collision_free_configeration_other = joinNode(collision_free_configeration,!is_goal);
                if (!collision_free_configeration_other.empty()){
                    if(collision_free_configeration == collision_free_configeration_other){
                        printf("Got a path\n");
                        reachedGoal = true;
                    }
                }
            }
            is_goal = !is_goal;
            if(j>max_iterations_){
                printf("Coundn't Find A Path\n");
                break;
            }
            j++;
        }
    }
    if(reachedGoal) {
        // printAngles(collision_free_configeration);
        // printAngles(tree_goal_[collision_free_configeration_other]);
        // printf("Goal\n");
        // printAngles(arm_goal_);
        printf("Getting Plan from Start\n");
        printf("%d\n",tree_.size());
        plan = getPath(collision_free_configeration);
        printf("%d\n",plan.size());
        printf("Getting Plan from Goal\n");
        path_to_goal = getPathToGoal(collision_free_configeration);
        printf("%d\n",tree_goal_.size());
        printf("%d\n",path_to_goal.size());
        printf("Got Plan from Goal\n");
        plan.insert(plan.end(), path_to_goal.begin(), path_to_goal.end());
        // total_cost_ = getPathCost(path);
    }
    else{
        total_cost_ = 0;
    }
}
void RRTConnect::replan(std::vector<std::vector<double>> &plan,const std::vector<double>& current_angle){
    arm_start_ = current_angle;
    tree_.clear();
    tree_goal_.clear();
    getFirstPlan(plan);
}


