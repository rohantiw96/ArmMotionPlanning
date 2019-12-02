#include "rrt_conn.h"

struct Node
{
    std::vector<double> angles_;
    double f_value_;
    Node()
    {
        f_value_ = 0;
    }
    Node(std::vector<double> angles, double f_value) : f_value_(f_value),angles_(angles) {}
};


struct CompareNode
{
    bool operator()(Node const &n1, Node const &n2)
    {
        return n1.f_value_ > n2.f_value_;
    }
};


RRTCONN::RRTCONN(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs)
    :SamplingPlanners(map,x_size,y_size,arm_start,arm_goal,numofDOFs){
        epsilon_ = 1;
        num_iteration_ = 100000;
        found_initial_path_ = false;

}


int RRTCONN::returnNumberOfVertices()
{
    return map.size();
}


double RRTCONN::getHeuristic(std::vector<double> current_node,std::vector<double> goal)
{
    return euclideanDistance(current_node,goal);
}


std::vector<std::vector<double>> RRTCONN::findKNearestNeighbor(const std::vector<double> &q_new){
    std::vector<std::vector<double>> k_nearest_neighbor;
    double euclidean_distance = 0;
        for(const auto& node:map){
            euclidean_distance = euclideanDistance(node.first,q_new);
            if(euclidean_distance < epsilon_){ //dont check for collisions
                k_nearest_neighbor.push_back(node.first);
            }
        }
    return k_nearest_neighbor;
}


std::vector<double> RRTCONN::findNearestNeighbor(const std::vector<double> &q_new){
    std::vector<double> nearest_neighbor;
    double euclidean_distance = 0;
    double min_distance = std::numeric_limits<double>::max();
    for(const auto& node:map){
        euclidean_distance = euclideanDistance(node.first,q_new);
        if(euclidean_distance < min_distance && interpolate(q_new,node.first)){
            min_distance = euclidean_distance;
            nearest_neighbor = node.first;
        }
    }
    printf("distance %f\n", min_distance);
    return nearest_neighbor;
}


void RRTCONN::addSample(std::vector<double> &q_new,std::vector<double> &q_neighbor){
    map[q_neighbor].push_back(q_new);
    map[q_new].push_back(q_neighbor);
}


void RRTCONN::removeNode(const std::vector<double> &current_angle)
{
    auto current_node = map.find(current_angle);
    int counter;
    for (auto &neighbor_nodes : current_node->second)
    {
        counter = 0;
        auto neighbor_nodes_neighbors =  &map.find(neighbor_nodes)->second;
        for (auto &neighbors_neighbor : *neighbor_nodes_neighbors)
        {
            if (neighbors_neighbor == current_angle)
            {
                neighbor_nodes_neighbors->erase(neighbor_nodes_neighbors->begin()+counter);
                break;
            }
            counter++;
        }
    }
    map.erase(current_node);
}


void RRTCONN::removeEdge(const std::vector<double> &current_angle,const std::vector<double> &next_angle)
{
    auto neighbor_nodes_neighbors = &map.find(current_angle)->second;
    int counter = 0;
    for (auto &neighbors_neighbor : *neighbor_nodes_neighbors)
    {
        if (neighbors_neighbor == next_angle)
        {
            neighbor_nodes_neighbors->erase(neighbor_nodes_neighbors->begin()+counter);
            break;
        }
        counter++;
    }

    neighbor_nodes_neighbors = &map.find(next_angle)->second;
    counter = 0;
    for (auto &neighbors_neighbor : *neighbor_nodes_neighbors)
    {
        if (neighbors_neighbor == current_angle)
        {
            neighbor_nodes_neighbors->erase(neighbor_nodes_neighbors->begin()+counter);
            break;
        }
        counter++;
    }
}


std::vector<std::vector<double>> RRTCONN::backTrack(std::vector<double> node, std::vector<double> start_neighbor,bool &found_collision_free_path){
    printf("backtracking\n");
    std::vector<double> current_angle = node;
    std::vector<std::vector<double>> path;
    bool success = true;
    //checking if all nodes are not in collision
    while (current_angle != start_neighbor) // Backtracking to get the shortest path
    {
        current_angle = came_from_[current_angle];
        if (!IsValidArmConfiguration(current_angle,true))
        {   
            removeNode(current_angle);
            success = false;
        }
        path.emplace_back(current_angle);
    }
    if (!success)
        return std::vector<std::vector<double>>{};

    std::reverse(path.begin(),path.end());

    // checking if interpolation between nodes are collision free
    for (int i = 0;i<path.size()-1;i++)
    {
        if (!interpolate(path[i],path[i+1]))
            {
                removeEdge(path[i],path[i+1]);
                return std::vector<std::vector<double>>{};
            }
    }
    found_collision_free_path = true;
    path.push_back(arm_goal_);
    path.insert(path.begin(),arm_start_);
    return path;
}


double RRTCONN::returnPathCost(){
    return total_cost_;
}


void RRTCONN::getFirstPlan(double ***plan,int *planlength){
    total_cost_= 0;
    std::vector<std::vector<double>> path = std::vector<std::vector<double>>{};
    if (!checkGoalAndStartForCollision()){
        buildRoadMap();
        path =  getShortestPath();
        if (path.size() > 0)
        {
            total_cost_ = getPathCost(path);
            found_initial_path_ = true;
            printf("found initial path\n");
        } 
    }
    else
    {
        printf("no path found\n");
    }
    
    returnPathToMex(path,plan,planlength);
}


void RRTCONN::replan(double ***plan,int *planlength,double *map,std::vector<double> current_angle){
    map_ = map;
    arm_start_ = current_angle;
    total_cost_= 0;
    std::vector<std::vector<double>> path = std::vector<std::vector<double>>{};
    if (!checkGoalAndStartForCollision()){
        path =  getShortestPath();
        if (path.size() > 0) total_cost_ = getPathCost(path);
    }
    printf("found replanned path\n");
    returnPathToMex(path,plan,planlength);
}