#pragma once
#include "sampling_planner.h"

class DRRT: public SamplingPlanners{
    public:
        DRRT(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs,double epsilon,int sampling_rate,double bias_probability,int max_iterations);
        void getFirstPlan(double ***plan,int *planlength);
        double returnPathCost();
        void replan(double ***plan,int *planlength,const std::vector<double>& current_angle);
        int returnNumberOfVertices();
    protected:
        double epsilon_;
        int num_samples_;
        double total_cost_;
        double bias_probability_;
        int max_iterations_;
        std::uniform_int_distribution<int> distribution_goal_selection_;
        std::mt19937 bias_generator_;
        std::unordered_map<std::vector<double>,std::vector<double>,container_hash<std::vector<double> > > tree_;
        std::unordered_map<std::vector<double>,std::vector<std::vector<double>>, container_hash<std::vector<double>>> child_map_;
        std::unordered_map<std::vector<double>,bool,container_hash<std::vector<double> > > invalid_nodes_;
        std::vector<double> findNearestNeighbor(const std::vector<double> &q_rand);
        std::vector<double> extend(const std::vector<double> &q_start,const std::vector<double> &q_end);
        void addNode(const std::vector<double>& parent,const std::vector<double>& child);
        bool inGoalRegion(const std::vector<double> &angles);
        std::vector<std::vector<double> > getPath(const std::vector<double>& start_angles,const std::vector<double>& goal_angles);
        std::vector<double> biasedAngleSampling(const double goal_bias_probability,const std::vector<double>& node);
        std::vector<double> interpolateBetweenNodes(const std::vector<double>& start,const std::vector<double>& end);
        void invalidateNodes();
        void trimNodes();
        void deleteAllChildNodes(const std::vector<double>& parent);
        void deleteEdge(const std::vector<double>& parent,const std::vector<double> child);
        bool regrowTree(const std::vector<double> current_angle);
};