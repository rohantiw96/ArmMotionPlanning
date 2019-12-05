#include "DRRT.h"

DRRT::DRRT(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int num_samples,double bias_probability,int max_iterations)
    :SamplingPlanners(map,x_size,y_size,arm_start,arm_goal,numofDOFs){
        epsilon_ = epsilon;
        num_samples_ = num_samples;
        bias_generator_ = std::mt19937(std::random_device()());
        distribution_goal_selection_ = std::uniform_int_distribution<int>(1, 100);
        bias_probability_ = bias_probability;
        max_iterations_ = max_iterations;
}

std::vector<double> DRRT::findNearestNeighbor(const std::vector<double> &q_rand){
    std::vector<double> nearest_neighbor;
    double min_distance = INT_MAX;
    double euclidean_distance = 0;
    for(const auto& n:tree_){
        euclidean_distance = euclideanDistance(n.first,q_rand);
        if(euclidean_distance < min_distance){
            min_distance = euclidean_distance;
            nearest_neighbor = n.first;
        }
    }
    return nearest_neighbor;
}
std::vector<double> DRRT::extend(const std::vector<double> &q_start,const std::vector<double> &q_end){
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

std::vector<double> DRRT::biasedAngleSampling(const double bias_probability,const std::vector<double>& node){
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

void DRRT::addNode(const std::vector<double> parent,const std::vector<double> child){
        tree_[child] = parent;
        child_map_[parent].push_back(child);
}

std::vector<double> DRRT::interpolateBetweenNodes(const std::vector<double>& start,const std::vector<double>& end){
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

bool DRRT::inGoalRegion(const std::vector<double> &angles){
    std::vector<double> diff_vector;
    for(int i=0;i<numofDOFs_;i++){
        diff_vector.push_back(angles[i] - arm_goal_[i]);
    }
    if (getNorm(diff_vector) <= epsilon_){
        return true;
    }
    return false;
}

std::vector<std::vector<double> > DRRT::getPath(const std::vector<double>& start_angles,const std::vector<double>& goal_angles){
    std::vector<std::vector<double> > path;
    std::vector<double> q_current = start_angles;
    while(tree_[q_current] != goal_angles){
        // printf("Angles: ");
        // for(const auto& n:q_current){
        //     printf("%f\n",n);
        // }
        // printf("Printed\n");
        path.push_back(q_current);
        q_current = tree_[q_current];
    }
    path.push_back(goal_angles);
    return path;
}

double DRRT::returnPathCost(){
    return total_cost_;
}

int DRRT::returnNumberOfVertices(){
    return tree_.size();
}

void DRRT::deleteEdge(const std::vector<double>& parent,const std::vector<double> child){
    for (int i=0;i<child_map_[parent].size();i++){
        if (child_map_[parent][i] == child){
            child_map_[parent].erase(child_map_[parent].begin()+i);
        }
    }
    tree_.erase(child);
    if(tree_.find(child) != tree_.end()){
        printf("Not Deleted\n");
    }
}

void DRRT::deleteAllChildNodes(const std::vector<double> parent){
    std::vector<double> current = parent;
    std::queue<std::vector<double>> que;
    que.push(current);
    while(!que.empty()){
        current = que.front();
        tree_.erase(current);
        que.pop();
        for(const auto n:child_map_[parent]){
            tree_.erase(n);
            que.push(n);
        }
        child_map_.erase(parent);
    } 
}

void DRRT::invalidateNodes(){
    printf("Size of Tree %ld\n",tree_.size());
    for(const auto& n:tree_){
        // if(!IsValidArmConfiguration(tree_[n],true))
        //     invalid_nodes_[tree_[n]] = true;
        // printf("Loop through Nodes\n");
        // printf("Child Size %d and Parent Size %d\n",n.first.size(),n.second.size());
        if (!IsValidArmConfiguration(n.first,true))
        {
            invalid_nodes_[n.first] = true;
        }
        else if (n.first != arm_goal_ && !interpolate(n.second,n.first)){
            invalid_nodes_[n.first] = true;
        }
    }
}

void DRRT::trimNodes(){
    for(const auto& node:invalid_nodes_){
        deleteEdge(tree_[node.first],node.first);
        deleteAllChildNodes(node.first);
    }
}
void DRRT::getFirstPlan(std::vector<std::vector<double>> &plan){
    std::vector<double> q_rand;
    std::vector<double> q_near;
    std::vector<double> q_epilison;
    std::vector<double> collision_free_configeration;
    bool reachedGoal = false;
    int count = 0;
    if (!checkGoalAndStartForCollision()){
        tree_[arm_goal_] = std::vector<double>{};
        while(!reachedGoal){ 
            q_rand = biasedAngleSampling(bias_probability_,arm_start_);
            q_near = findNearestNeighbor(q_rand);
            q_epilison = extend(q_near,q_rand);
            collision_free_configeration = interpolateBetweenNodes(q_near,q_epilison);
            if (collision_free_configeration != q_near){
                addNode(q_near,collision_free_configeration);
                if(collision_free_configeration == arm_start_){
                    printf("Found A Path\n");
                    reachedGoal = true;
                }
            }
            if(count > max_iterations_){
                printf("No Path Found During First Plan, Max iterations exceeded\n");
                break;
            }
            count++;
        }
    }
    if(reachedGoal) {
        printf("BackTracking\n");
        plan = getPath(arm_start_,arm_goal_);
        total_cost_ = getPathCost(plan);
        // printf("Total Cost %f\n",total_cost_);
    }
    else{
        total_cost_ = 0;
    }
}

bool DRRT::regrowTree(const std::vector<double> current_angle){
    bool reachedCurrentState = false;
    std::vector<double> q_rand;
    std::vector<double> q_near;
    std::vector<double> q_epilison;
    std::vector<double> collision_free_configeration;
    arm_start_ = current_angle;
    printf("Tree Size %ld\n",tree_.size());
    int count = 0;
    if (!checkGoalAndStartForCollision()){
        while(!reachedCurrentState){
            q_rand = biasedAngleSampling(bias_probability_,current_angle);
            q_near = findNearestNeighbor(q_rand);
            q_epilison = extend(q_near,q_rand);
            collision_free_configeration = interpolateBetweenNodes(q_near,q_epilison);
            if (collision_free_configeration != q_near){
                // printf("Adding Node\n");
                addNode(q_near,collision_free_configeration);
                if(collision_free_configeration == current_angle){
                    printf("Found A Path\n");
                    reachedCurrentState = true;
                }
            }
            if(count > max_iterations_){
                printf("No Path Found During Replanning, Max iterations exceeded\n");
                return false;
            }
            count++;
        }
        return true;
    }
    else{
        printf("Current Position in Collision\n");
        return false;
    } 
}
void DRRT::replan(std::vector<std::vector<double>> &plan,const std::vector<double>& current_angle){
    printf("REPLANNING DRRT\n");
    invalidateNodes();
    printf("Invalidated Nodes\n");
    trimNodes();
    printf("Trimed Nodes\n");
    if(regrowTree(current_angle)){
        printf("Regrown Tree\n");
        plan = getPath(current_angle,arm_goal_);
    }
}