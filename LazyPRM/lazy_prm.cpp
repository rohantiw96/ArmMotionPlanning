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

std::vector<std::vector<double>> LAZYPRM::findKNearestNeighbor(const std::vector<double> &q_new){
    std::vector<std::vector<double>> k_nearest_neighbor;
    double euclidean_distance = 0;
    for(const auto& c:comopnents_){
        for(const auto& m:c){
            euclidean_distance = euclideanDistance(m.first,q_new);
            if(euclidean_distance < epsilon_){    // && interpolate(q_new,m.first)){ //dont check for collisions
                k_nearest_neighbor.push_back(m.first);
            }
        }
    }
    return k_nearest_neighbor;
}

std::vector<double> LAZYPRM::findNearestNeighbor(const std::vector<double> &q_new){
    std::vector<double> nearest_neighbor;
    double euclidean_distance = 0;
    double min_distance = std::numeric_limits<double>::max();
    for(const auto& c:comopnents_){
        for(const auto& m:c){
            euclidean_distance = euclideanDistance(m.first,q_new);
            if(euclidean_distance < min_distance && interpolate(q_new,m.first)){
                min_distance = euclidean_distance;
                nearest_neighbor = m.first;
            }
        }
    }
    return nearest_neighbor;
}

std::vector<double> LAZYPRM::findNearestNeighborComponenent(const std::vector<double> &q_new, component_map &comp){
    std::vector<double> nearest_neighbor;
    double euclidean_distance = 0;
    double min_distance = std::numeric_limits<double>::max();
    for(const auto& nodes:comp){
        euclidean_distance = euclideanDistance(nodes.first,q_new);
        if(euclidean_distance < min_distance && interpolate(q_new,nodes.first)){
            min_distance = euclidean_distance;
            nearest_neighbor = nodes.first;
        }
    }
    return nearest_neighbor;
}

void LAZYPRM::addSample(std::vector<double> &q_new,std::vector<double> &q_neighbor){
    for(auto& c:comopnents_){
        if(c.find(q_neighbor) != c.end()){
            c[q_neighbor].push_back(q_new);
            c[q_new].push_back(q_neighbor);
            break;
        }
    }
}
int LAZYPRM::findComponent(std::vector<double> &q_neighbor){
    int num_component = 0;
    for(auto& c:comopnents_){
        if(c.find(q_neighbor) != c.end()){
            break;
        }
        num_component++;
    }
    return num_component;
}
void LAZYPRM::mergeComponents(component_map &m1,component_map &m2){
    if(m1.size() > m2.size()) {
        m1.insert(m2.begin(),m2.end());
        comopnents_.remove(m2);
    }
    else {
        m2.insert(m1.begin(),m1.end());
        comopnents_.remove(m1);
    }
}
std::vector<std::vector<double>> LAZYPRM::getStartAndGoalNode()
{
    std::vector<double> start_neighbor = findNearestNeighbor(arm_start_);
    std::vector<double> goal_neighbor = findNearestNeighbor(arm_goal_);
    return std::vector<std::vector<double>>{start_neighbor,goal_neighbor};
}

void LAZYPRM::removeNode(const std::vector<double> &current_angle, component_map &m)
{
    printf("removing Node\n");
    auto current_node = m.find(current_angle);
    int counter=0;
    for (auto &neighbor_nodes : current_node->second)
    {
        printf("looping through neighbors\n");
        counter = 0;
        auto neighbor_nodes_neighbors =  &m.find(neighbor_nodes)->second;
        for (auto &neighbors_neighbor : *neighbor_nodes_neighbors)
        {
            printf("looping through neighbors neighbors\n");
            if (neighbors_neighbor == current_angle)
            {
                printf("deleting neighbors\n");
                neighbor_nodes_neighbors->erase(neighbor_nodes_neighbors->begin()+counter);
                printf("done deleting\n");
                break;
            }
            counter++;
        }
    }
    m.erase(current_node);
}

void LAZYPRM::removeEdge(const std::vector<double> &current_angle,const std::vector<double> &next_angle, component_map &m)
{
    printf("removing Edge\n");
    auto neighbor_nodes_neighbors = m.find(current_angle)->second;
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

    neighbor_nodes_neighbors = m.find(next_angle)->second;
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

std::vector<std::vector<double>> LAZYPRM::backTrack(std::vector<double> node, std::vector<double> start_neighbor,bool &found_collision_free_path,component_map &m){
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
                removeNode(current_angle,m);
                if (m.find(current_angle)==m.end())
                    printf("deleted node correctly");
                for (auto nodes : m)
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
    for (int i = 0;i<path.size()-1;i++) // Backtracking to get the shortest path
    {
        if (!interpolate(path[i],path[i+1]))
            {
                removeEdge(path[i],path[i+1],m);
                return std::vector<std::vector<double>>{};
            }
    }
    found_collision_free_path = true;
    path.push_back(arm_goal_);
    path.insert(path.begin(),arm_start_);
    return path;
}

int LAZYPRM::returnNumberOfVertices(){
    return (*std::next(comopnents_.begin(), findComponent(arm_goal_))).size();
}

std::vector<std::vector<double>> LAZYPRM::getShortestPath(){
    std::vector<std::vector<double>> final_path;
    auto temp = getStartAndGoalNode();
    std::vector<double> start_neighbor = temp[0];
    std::vector<double> goal_neighbor = temp[1];
    int goal_component = findComponent(start_neighbor);
    if (goal_component != findComponent(goal_neighbor)){
        printf("Start and Goal don't belong to the same components\n");
        return std::vector<std::vector<double>>{};
    }
    else{
        printf("starting to check for collisions\n");
        component_map m = *std::next(comopnents_.begin(), goal_component);
        bool found_collision_free_path = false;
        while (!found_collision_free_path)
        {
            start_neighbor = findNearestNeighborComponenent(arm_start_,m);
            goal_neighbor = findNearestNeighborComponenent(arm_goal_,m);
            std::cout<<"start angle\n";
            printAngles(start_neighbor);
            std::cout<<"goal angle\n";
            printAngles(goal_neighbor);
            printf("componenent size %ld ",m.size());
            came_from_.clear();
            final_path.clear();
            if (m.find(goal_neighbor)==m.end() || m.find(start_neighbor)==m.end())
            {
                printf("removed start or goal node connection\n");
                found_collision_free_path == true;
                break;
            }
            std::unordered_map<std::vector<double>,double,container_hash<std::vector<double>>> dijkstra_cost_;
            for(const auto& nodes:m){
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
                neighbors = m.find(current)->second;
                for(const auto& n:neighbors){
                    cost = euclideanDistance(current,n) + dijkstra_cost_[current];
                    if (cost < dijkstra_cost_[n]){
                        dijkstra_cost_[n] = cost;
                        list.push(n);
                        came_from_[n] = current;
                    }
                }
            }
            printf("here 9\n");
            final_path = backTrack(goal_neighbor,start_neighbor,found_collision_free_path,m);
            printf("componenent size  after backtracking %ld ",m.size());
        }
    }
    return final_path;
}

void LAZYPRM::buildRoadMap(){
    int iter = 0;
    std::vector<double> q_rand;
    std::vector<std::vector<double>> k_nearest_neighbors;
    int own_component = -1;
    int neighbor_component = -1;
    while(iter < num_iteration_){
        q_rand = getRandomAngleConfig(0,std::vector<double>{});
        if (IsValidArmConfiguration(q_rand,false)){
            k_nearest_neighbors = findKNearestNeighbor(q_rand);
            if(k_nearest_neighbors.size()>0){
                addSample(q_rand,*k_nearest_neighbors.begin()); // Add the sample to the componenent of the first neighbor
                own_component = findComponent(q_rand);
                for (auto neighbors = std::next(k_nearest_neighbors.begin()); neighbors != k_nearest_neighbors.end(); ++neighbors){
	                neighbor_component = findComponent(*neighbors);
                    if(own_component!=neighbor_component){
                        mergeComponents(*std::next(comopnents_.begin(), own_component),*std::next(comopnents_.begin(), neighbor_component));
                        addSample(q_rand,*neighbors);
                        own_component = findComponent(q_rand);
                    }
                }   
            }
            else{
                std::unordered_map<std::vector<double>, std::vector<std::vector<double>>, container_hash<std::vector<double>>> new_component;
                new_component[q_rand] = std::vector<std::vector<double>>{};
                comopnents_.emplace_back(new_component);
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
