#pragma once
#include <iostream>
#include <math.h>
#include <random>
#include <chrono>
#include <boost/functional/hash.hpp>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <functional>
#include <cmath>
#include <queue>
#define PI 3.141592654

typedef struct
{
    int X1, Y1;
    int X2, Y2;
    int Increment;
    int UsingYIndex;
    int DeltaX, DeltaY;
    int DTerm;
    int IncrE, IncrNE;
    int XIndex, YIndex;
    int Flipped;
} bresenham_param_t;

template <typename Container> 
struct container_hash {
    std::size_t operator()(Container const& c) const {
        return boost::hash_range(c.begin(), c.end());
    }
};

class SamplingPlanners
{
public:
    SamplingPlanners(double *map,
                     int x_size,
                     int y_size,
                     const std::vector<double> &arm_start,
                     const std::vector<double> &arm_goal,
                     int numofDOFs);

    bool interpolate(const std::vector<double> &start,const std::vector<double> &end);
    void updateMap(double *map);
    int IsValidArmConfiguration(std::vector<double> angles,bool checkCollision);
    void printAngles(std::vector<double> angles);



protected:
    double *map_;
    int x_size_;
    int y_size_;
    std::vector<double> arm_start_;
    std::vector<double> arm_goal_;
    int numofDOFs_;
    int num_samples_;
    double ***plan_;
    int *planlength_;
    std::random_device rand_device_;
    std::mt19937 generator_;
    std::uniform_real_distribution<double> angle_distribution_;
    
    void ContXY2Cell(const double x, const double y, short unsigned int *pX, short unsigned int *pY);
    void get_bresenham_parameters(const int p1x, const int p1y, const int p2x, const int p2y, bresenham_param_t *params);
    void get_current_point(bresenham_param_t *params, int *x, int *y);
    int get_next_point(bresenham_param_t *params);
    int IsValidLineSegment(const double x0, const double y0, const double x1, const double y1,bool checkCollision);
    int getMapIndex(const int x, const int y);
    double euclideanDistance(const std::vector<double> &q_1,const std::vector<double> &q_2);
    double getNorm(const std::vector<double>& vec);
    void wrapAngles(std::vector<double> &angles);
    void returnPathToMex(const std::vector<std::vector<double>>& path,double ***plan,int *planlength);
    double getPathCost(const std::vector<std::vector<double>>& path);
    bool checkGoalAndStartForCollision();
    std::vector<double> getRandomAngleConfig();
};
