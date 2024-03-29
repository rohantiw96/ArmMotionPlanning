#pragma once
#include "sampling_planner.h"
#include <queue>

typedef std::unordered_map<std::vector<double>,std::vector<std::vector<double>>,container_hash<std::vector<double>>> component_map;

class RRTCONN: public SamplingPlanners{
    public:
        RRTCONN(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs);
        void getFirstPlan(double ***plan,int *planlength);
        void replan(double ***plan,int *planlength,double *map,std::vector<double> current_angle);
        double returnPathCost();
        int returnNumberOfVertices();
    private:
        double epsilon_;
        int num_iteration_;
        int num_samples_;
        double total_cost_;
        bool found_initial_path_;
        std::vector<std::vector<double>> findKNearestNeighbor(const std::vector<double> &q_new);
        void addSample(std::vector<double> &q_new,std::vector<double> &q_neighbor);
        std::vector<double> findNearestNeighbor(const std::vector<double> &q_new);
        std::vector<std::vector<double>> backTrack(std::vector<double> node, std::vector<double> start_neighbor,bool &found_collision_free_path);
        void removeNode(const std::vector<double> &current_angle);
        void removeEdge(const std::vector<double> &current_angle,const std::vector<double> &next_angle);
        double getHeuristic(std::vector<double> current_node,std::vector<double> goal);
};
