#pragma once
#include "sampling_planner.h"
#include <queue>

typedef std::unordered_map<std::vector<double>,std::vector<std::vector<double>>,container_hash<std::vector<double>>> component_map;

class LAZYPRM: public SamplingPlanners{
    public:
        LAZYPRM(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs);
        void plan(double ***plan,int *planlength);
        void buildRoadMap();
        double returnPathCost();
        int returnNumberOfVertices();
    private:
        component_map map;
        double epsilon_;
        int num_iteration_;
        int num_samples_;
        double total_cost_;
        std::unordered_map<std::vector<double>,std::vector<double>,container_hash<std::vector<double>>> came_from_;
        std::vector<std::vector<double>> findKNearestNeighbor(const std::vector<double> &q_new);
        bool interpolate(const std::vector<double> &start,const std::vector<double> &end);
        void addSample(std::vector<double> &q_new,std::vector<double> &q_neighbor);
        std::vector<double> findNearestNeighbor(const std::vector<double> &q_new);
        std::vector<std::vector<double>> getShortestPath();
        std::vector<std::vector<double>> backTrack(std::vector<double> node, std::vector<double> start_neighbor,bool &found_collision_free_path);
        void removeNode(const std::vector<double> &current_angle);
        void removeEdge(const std::vector<double> &current_angle,const std::vector<double> &next_angle);

};
