#pragma once
#include "sampling_planner.h"

class RRTConnect: public SamplingPlanners{
    public:
        RRTConnect(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int sampling_rate,double bias_probability,int max_iterations);
        void getFirstPlan(std::vector<std::vector<double>> &plan);
        void replan(std::vector<std::vector<double>> &plan,const std::vector<double>& current_angle);
    private:
        double epsilon_;
        int sampling_rate_;
        double total_cost_;
        double bias_probability_;
        int max_iterations_;
        std::uniform_int_distribution<int> distribution_goal_selection_;
        std::mt19937 bias_generator_;
        std::unordered_map<std::vector<double>,std::vector<double>,container_hash<std::vector<double> > > tree_;
        std::unordered_map<std::vector<double>,std::vector<double>,container_hash<std::vector<double> > > tree_goal_;
        void addNode(const std::vector<double> &q_near,const std::vector<double> &q_new,bool is_goal);
        std::vector<double> biasedAngleSampling(const double bias_probability,const std::vector<double>& node);
        std::vector<double> findNearestNeighbor(const std::vector<double> &q_rand,const bool is_goal);
        std::vector<double> extendNode(std::vector<double> q_new,bool is_goal);
        std::vector<double> joinNode(std::vector<double> q_new,bool is_goal);
        std::vector<std::vector<double>> getPathToGoal(const std::vector<double> &angles);
        std::vector<double> extend(const std::vector<double> &q_start,const std::vector<double> &q_end);
        std::vector<double> interpolateBetweenNodes(const std::vector<double>& start,const std::vector<double>& end);
        std::vector<std::vector<double> > getPath(const std::vector<double> &start);
};