#include "sampling_planner.h"
// #include "mex.h"

#define LINKLENGTH_CELLS 10
#define PI 3.141592654
SamplingPlanners::SamplingPlanners(double *map,int x_size,int y_size,const std::vector<double> &arm_start,const std::vector<double> &arm_goal,int numofDOFs){
  map_ = map;
  x_size_ = x_size;
  y_size_ = y_size;
  arm_goal_ = arm_goal;
  arm_start_ = arm_start;
  numofDOFs_ = numofDOFs;
  generator_ = std::mt19937(std::random_device()());
  angle_distribution_ = std::uniform_real_distribution<double>(0, 2*PI);
  num_samples_ = 200;
};

void SamplingPlanners::updateMap(double *map)
{
  map_ = map;
}

bool SamplingPlanners::interpolate(const std::vector<double> &start,const std::vector<double> &end){
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

void SamplingPlanners::printAngles(std::vector<double> angles)
{
  for (auto angle : angles)
  {
    printf("%f ",angle);
  }
  printf("\n");
}

void SamplingPlanners::ContXY2Cell(const double x, const double y, short unsigned int *pX, short unsigned int *pY)
{
  double cellsize = 1.0;
  //take the nearest cell
  *pX = (int)(x / (double)(cellsize));
  if (x < 0)
    *pX = 0;
  if (*pX >= x_size_)
    *pX = x_size_ - 1;

  *pY = (int)(y / (double)(cellsize));
  if (y < 0)
    *pY = 0;
  if (*pY >= y_size_)
    *pY = y_size_ - 1;
}

int SamplingPlanners::getMapIndex(const int x, const int y)
{
  return y * x_size_ + x;
}

void SamplingPlanners::get_bresenham_parameters(const int p1x, const int p1y, const int p2x, const int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y - p1y) / (double)(p2x - p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
  {
    params->Y1 = p1x;
    params->X1 = p1y;
    params->Y2 = p2x;
    params->X2 = p2y;
  }
  else
  {
    params->X1 = p1x;
    params->Y1 = p1y;
    params->X2 = p2x;
    params->Y2 = p2y;
  }

  if ((p2x - p1x) * (p2y - p1y) < 0)
  {
    params->Flipped = 1;
    params->Y1 = -params->Y1;
    params->Y2 = -params->Y2;
  }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX = params->X2 - params->X1;
  params->DeltaY = params->Y2 - params->Y1;

  params->IncrE = 2 * params->DeltaY * params->Increment;
  params->IncrNE = 2 * (params->DeltaY - params->DeltaX) * params->Increment;
  params->DTerm = (2 * params->DeltaY - params->DeltaX) * params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void SamplingPlanners::get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
  {
    *y = params->XIndex;
    *x = params->YIndex;
    if (params->Flipped)
      *x = -*x;
  }
  else
  {
    *x = params->XIndex;
    *y = params->YIndex;
    if (params->Flipped)
      *y = -*y;
  }
}

int SamplingPlanners::get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
  {
    return 0;
  }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
  {
    params->DTerm += params->IncrNE;
    params->YIndex += params->Increment;
  }
  return 1;
}

int SamplingPlanners::IsValidLineSegment(const double x0, const double y0, const double x1, const double y1, bool checkCollision)

{
  bresenham_param_t params;
  int nX, nY;
  short unsigned int nX0, nY0, nX1, nY1;

  //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);

  //make sure the line segment is inside the environment
  if (x0 < 0 || x0 >= x_size_ ||
      x1 < 0 || x1 >= x_size_ ||
      y0 < 0 || y0 >= y_size_ ||
      y1 < 0 || y1 >= y_size_)
    return 0;
  
  if (!checkCollision) //dont check for obstacle collision
    return 1;

  ContXY2Cell(x0, y0, &nX0, &nY0);
  ContXY2Cell(x1, y1, &nX1, &nY1);

  //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

  //iterate through the points on the segment
  get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
  do
  {
    get_current_point(&params, &nX, &nY);
    if (map_[getMapIndex(nX, nY)] == 1)
      return 0;
  } while (get_next_point(&params));

  return 1;
}

int SamplingPlanners::IsValidArmConfiguration(std::vector<double> angles,bool checkCollision)
{
  double x0, y0, x1, y1;
  int i;

  //iterate through all the links starting with the base
  x1 = ((double)x_size_) / 2.0;
  y1 = 0;
  for (i = 0; i < numofDOFs_; i++)
  {
    //compute the corresponding line segment
    x0 = x1;
    y0 = y1;
    x1 = x0 + LINKLENGTH_CELLS * cos(2 * PI - angles[i]);
    y1 = y0 - LINKLENGTH_CELLS * sin(2 * PI - angles[i]);

    //check the validity of the corresponding line segment
    if (!IsValidLineSegment(x0, y0, x1, y1,checkCollision))
      return 0;
  }
  return 1;
}
double SamplingPlanners::getNorm(const std::vector<double>& vec){
    double norm = 0;
    for(const auto& i:vec){
        norm += i*i;
    }
    return std::sqrt(norm);
}
double SamplingPlanners::euclideanDistance(const std::vector<double> &q_1,const std::vector<double> &q_2){
    std::vector<double> diff;
    for(int i = 0;i <numofDOFs_;i++){
        diff.push_back(q_2[i] - q_1[i]);
    }
    return getNorm(diff);
}

void SamplingPlanners::wrapAngles(std::vector<double> &angles){
    for(int i=0;i<angles.size();i++){
        angles[i] = fmod(angles[i],2*PI);
        if (angles[i] < 0)
            angles[i] += 2*PI;
    }
}

std::vector<double> SamplingPlanners::getRandomAngleConfig(){
  std::vector<double> angles;
  for(int i=0;i<numofDOFs_;i++){
      angles.push_back(angle_distribution_(generator_));
  }
  return angles;
}

void SamplingPlanners::returnPathToMex(const std::vector<std::vector<double>>& path,double ***plan,int *planlength){
    *plan = NULL;
    *planlength = path.size();
    if(*planlength > 0){
        *plan = (double**) malloc(*planlength*sizeof(double*));
        for (int i = 0; i < *planlength; i++){
            (*plan)[i] = (double*) malloc(numofDOFs_*sizeof(double)); 
            for(int j = 0; j < numofDOFs_; j++){
                (*plan)[i][j] = path[i][j];
            }
        }
    }
    printf("path given to mex\n");
}
double SamplingPlanners::getPathCost(const std::vector<std::vector<double>>& path){
    double total_cost = 0;
    for(int i=0;i<path.size()-1;i++){
        total_cost = total_cost + euclideanDistance(path[i],path[i+1]);
    }
    return total_cost; 
}

bool SamplingPlanners::checkGoalAndStartForCollision(){
  if (!IsValidArmConfiguration(arm_goal_,true))
    {
      printf("goal point is in collision\n");
      return true;
    }
  if (!IsValidArmConfiguration(arm_start_,true))
    {
      printf("starting point is in collision\n");
      return true;
    }
  return false;
}