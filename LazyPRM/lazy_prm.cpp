#include "lazy_prm.h"

LAZYPRM::LAZYPRM(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs)
    :SamplingPlanners(map,x_size,y_size,arm_start,arm_goal,numofDOFs){
        epsilon_ = 1;
        num_iteration_ = 100000;
        num_samples_ = 5;
}
bool LAZYPRM::interpolate(const std::vector<double> &start,const std::vector<double> &end){
    std::vector<double> delta;
    for(int i=0;i<numofDOFs_;i++){
        delta.push_back((end[i] - start[i])/ (num_samples_ - 1));
    }
    for(int i=0; i < num_samples_- 1; i++){
        std::vector<double> angles;
        for(int j=0;j<numofDOFs_;j++){
            angles.push_back(start[j]+ delta[j] * i);
        }
        if (!IsValidArmConfiguration(angles,true)){
            return false;
        }
    }
    if (!IsValidArmConfiguration(end,true))
        return false;

    return true;
}
int LAZYPRM::returnNumberOfVertices()
{
    return map.size();
}

std::vector<std::vector<double>> LAZYPRM::findKNearestNeighbor(const std::vector<double> &q_new){
    std::vector<std::vector<double>> k_nearest_neighbor;
    double euclidean_distance = 0;
        for(const auto& node:map){
            euclidean_distance = euclideanDistance(node.first,q_new);
            if(euclidean_distance < epsilon_){    // && interpolate(q_new,m.first)){ //dont check for collisions
                k_nearest_neighbor.push_back(node.first);
            }
        }
    return k_nearest_neighbor;
}

std::vector<double> LAZYPRM::findNearestNeighbor(const std::vector<double> &q_new){
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
    return nearest_neighbor;
}

void LAZYPRM::addSample(std::vector<double> &q_new,std::vector<double> &q_neighbor){
    map[q_neighbor].push_back(q_new);
    map[q_new].push_back(q_neighbor);
}

void LAZYPRM::removeNode(const std::vector<double> &current_angle)
{
    auto current_node = map.find(current_angle);
    int counter=0;
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

void LAZYPRM::removeEdge(const std::vector<double> &current_angle,const std::vector<double> &next_angle)
{
    auto neighbor_nodes_neighbors = map.find(current_angle)->second;
    int counter = 0;
    for (auto &neighbors_neighbor : neighbor_nodes_neighbors)
    {
        if (neighbors_neighbor == next_angle)
        {
            neighbor_nodes_neighbors.erase(neighbor_nodes_neighbors.begin()+counter);
            break;
        }
        counter++;
    }

    neighbor_nodes_neighbors = map.find(next_angle)->second;
    counter = 0;
    for (auto &neighbors_neighbor : neighbor_nodes_neighbors)
    {
        if (neighbors_neighbor == current_angle)
        {
            neighbor_nodes_neighbors.erase(neighbor_nodes_neighbors.begin()+counter);
            break;
        }
        counter++;
    }
}

std::vector<std::vector<double>> LAZYPRM::backTrack(std::vector<double> node, std::vector<double> start_neighbor,bool &found_collision_free_path){
    printf("back tracking called\n");
    std::vector<double> current_angle = node;
    std::vector<std::vector<double>> path;

    //checking if all nodes are not in collision
    while (current_angle != start_neighbor) // Backtracking to get the shortest path
    {
        printf("current angle\n");
        printAngles(current_angle);
        current_angle = came_from_[current_angle];
        printf("printing current node\n");
        for (auto angle : current_angle)
        {
            printf("%f ",angle);
        } 
        printf("\n");
        if (!IsValidArmConfiguration(current_angle,true))
        {   
            printf("deleting node\n");           
            for (auto angle : current_angle)
            {
                printf("%f ",angle);
            } 
                printf("\n");
                removeNode(current_angle);
                if (map.find(current_angle)==map.end())
                    printf("deleted node correctly");
                for (auto nodes : map)
                {
                    for (auto neighbor : nodes.second)
                    {
                        if (neighbor == current_angle)
                            printf("ERROOORRRRRR\n");
                    }

                }
                return std::vector<std::vector<double>>{};
            }
        else
        {
            printf("found valid node\n");
        }
        
        path.emplace_back(current_angle);
    }
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

std::vector<std::vector<double>> LAZYPRM::getShortestPath(){
    std::vector<std::vector<double>> final_path;
    std::vector<double> start_neighbor;
    std::vector<double> goal_neighbor;
    bool found_collision_free_path = false;
    while (!found_collision_free_path)
    {
        start_neighbor = findNearestNeighbor(arm_start_);
        goal_neighbor = findNearestNeighbor(arm_goal_);
        came_from_.clear();
        final_path.clear();
        if (map.find(goal_neighbor)==map.end() || map.find(start_neighbor)==map.end())
        {
            printf("removed start or goal node connection\n");
            found_collision_free_path == true;
            break;
        }
        std::unordered_map<std::vector<double>,double,container_hash<std::vector<double>>> dijkstra_cost_;
        for(const auto& nodes:map){
            dijkstra_cost_[nodes.first] = std::numeric_limits<double>::max();
        }
        std::priority_queue<std::vector<double>> list;
        dijkstra_cost_[start_neighbor] = 0.0; // Cost of tbe initial node 0
        list.push(start_neighbor);
        double cost = 0;
        std::vector<std::vector<double>> neighbors;
        std::vector<double> current;
        while(!list.empty()){
            current = list.top();
            list.pop();
            if (current == goal_neighbor)
            {
                printf("goal found in dijkstra\n");
                break;
            }
            neighbors = map.find(current)->second;
            for(const auto& n:neighbors){
                cost = euclideanDistance(current,n) + dijkstra_cost_[current];
                if (cost < dijkstra_cost_[n]){
                    dijkstra_cost_[n] = cost;
                    list.push(n);
                    came_from_[n] = current;
                }
            }
        }
        final_path = backTrack(goal_neighbor,start_neighbor,found_collision_free_path);
    }
    return final_path;
}

void LAZYPRM::buildRoadMap(){
    int iter = 0;
    std::vector<double> q_rand;
    std::vector<std::vector<double>> k_nearest_neighbors;
    int initialized = false;
    while(iter < num_iteration_){
        q_rand = getRandomAngleConfig(0,std::vector<double>{});
        if (IsValidArmConfiguration(q_rand,false)){
            k_nearest_neighbors = findKNearestNeighbor(q_rand);
            if(k_nearest_neighbors.size()>0){
                addSample(q_rand,*k_nearest_neighbors.begin());
                for(auto neighbors = std::next(k_nearest_neighbors.begin()); neighbors != k_nearest_neighbors.end(); ++neighbors){
                        addSample(q_rand,*neighbors);
                }
            }
            else if (!initialized && k_nearest_neighbors.size()==0)
            {
                std::vector<std::vector<double>> empty_neighbor{};
                map[q_rand] = empty_neighbor;
                initialized = true;
            }
               
        }
        iter++;
    }
    printf("built map\n");
}
double LAZYPRM::returnPathCost(){
    return total_cost_;
}
void LAZYPRM::plan(double ***plan,int *planlength){
    total_cost_= 0;
    std::vector<std::vector<double>> path = std::vector<std::vector<double>>{};
    if (!checkGoalAndStartForCollision()){
        buildRoadMap();
        path =  getShortestPath();
        if (path.size() > 0) total_cost_ = getPathCost(path);
    }
    returnPathToMex(path,plan,planlength);
}
