#include <math.h>
#include "mex.h"
#include <time.h>
#include "include/sampling_planner.h"
#include "LazyPRM/lazy_prm.h"
#include "DRRT/DRRT.h"
// #include "RRTConn/rrt_conn.h"
#include <iostream>
#include <vector>
#include <chrono>


/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]
#define	MAP_INFLATED_IN      prhs[4]

/* Planner Ids */

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]
#define	PATHTIME_OUT	plhs[2]
#define	PATHCOST_OUT	plhs[3]
#define	NUMVERTICES_OUT	plhs[4]


// #define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10

bool increment_arm(std::vector<double>& arm_next, const std::vector<double>& arm_current, double maxjntspeed, std::vector<double> &next_plan, int numofDOFs)
{
    double scale_factor = 1.;
    for(int idx=0; idx < numofDOFs; idx++){
        double delta = abs(next_plan[idx] - arm_current[idx]);
        if(delta > maxjntspeed){
            double new_factor = maxjntspeed / delta;
            if(new_factor < scale_factor){
                scale_factor = new_factor;
            }
        }
    }
    for(int idx=0; idx < numofDOFs; idx++){
        arm_next[idx] = arm_current[idx] + (next_plan[idx] - arm_current[idx])*scale_factor;
    }

    if(scale_factor == 1){
        return true;
    }
    else{
        return false;
    }
}


static void planner(
        int planner_id,
        double*	map,
        int x_size,
        int y_size,
        int t_size,
        double* armstart_anglesV_rad,
        double* armgoal_anglesV_rad,
        int numofDOFs,
        double*** plan,
        int* planlength)
{

    return;
}

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    /* Check for proper number of arguments */    
    if (nrhs != 5) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Five input arguments required."); 
    } else if (nlhs != 5) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    int t_size = y_size/x_size;
    y_size = x_size;
    double* map = mxGetPr(MAP_IN);
    double* map_inflated = mxGetPr(MAP_INFLATED_IN);
    
    /* get the start and goal angles*/     
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");         
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");         
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);
    std::vector<double> arm_start(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    std::vector<double> arm_goal(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);
    
    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }
    
    //plan variables assignment
    std::vector<std::vector<double>> plan{};

    //keep track of arm trajectory
    //TODO: implement this and return to matlab instead of plan
    std::vector<std::vector<double>> traj_vector{arm_start};
    
    //Tunable parameters
    int lookahead = 5;
    double maxjntspeed = 0.3;
    
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();    
    double* maplayer = &map[2500];
    double* maplayer_inflated = &map_inflated[2500];
    printf("map xdim: %d, ydim: %d, tdim: %d\n", x_size, y_size, t_size);

    SamplingPlanners planner_inflated(map_inflated,x_size,y_size,arm_start,arm_goal,numofDOFs);

    //params for DRRT
    double epsilon = 0.5;
    double interpolation_sampling = 50;
    double goal_bias_probability = 0.1;
    int max_iterations = 50000;
    DRRT planner(map,x_size,y_size,arm_start,arm_goal,numofDOFs,epsilon,interpolation_sampling,goal_bias_probability,max_iterations);

    // LAZY PRM
    // LAZYPRM planner(map,x_size,y_size,arm_start,arm_goal,numofDOFs);
    
    std::chrono::high_resolution_clock::time_point t_startplan = std::chrono::high_resolution_clock::now();
    planner.getFirstPlan(plan);
    std::chrono::high_resolution_clock::time_point t_endplan = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> t_plandiff = t_endplan - t_startplan;
    double t_plan = t_plandiff.count()/1000.0;
    printf("Total length %ld\n",plan.size());
    printf("first plan took %f seconds\n", t_plan);

    int layersize = x_size * y_size;
    std::vector<double> arm_current = arm_start;
    std::vector<double> arm_next = arm_current;
    std::vector<double> arm_future = arm_next;
    int next_plan_step = 1;

    for(int t=0; t < t_size; t++){
        printf("timestep: %d\n", t);
        int layer_index = layersize * t;
        maplayer = &map[layer_index];
        maplayer_inflated = &map_inflated[layer_index];
        planner_inflated.updateMap(maplayer_inflated);

        if(plan.size()==0){
            break;
        }

        // Check for collisions in the future of trajectory
        bool notcollision = true;
        int future_plan_step = next_plan_step;
        for(int t_future = 0; t_future < lookahead; t_future++){
            bool plan_step_reached = increment_arm(arm_future, arm_next, maxjntspeed, plan[future_plan_step], numofDOFs);
            notcollision = planner_inflated.interpolate(arm_future, arm_next); 

            if(!notcollision){
                printf("COLLISION FOUND\n");
                break;
            }
            if(plan_step_reached && future_plan_step < plan.size()-1){
                future_plan_step++;
            }
            else
            {
                break;
            }            
            arm_next = arm_future;
        }

        //Increment if no collision
        printf("not in collision: %d\n", notcollision);
        if(!notcollision){
            // Check if arm_current is in collision
            if(!planner_inflated.IsValidArmConfiguration(arm_current, true)){
                printf("ARM IS IN COLLISION WITH INFLATED MAP");
                break;
            }

            printf("REPLANNING\n");
            plan.clear();
            t_startplan = std::chrono::high_resolution_clock::now();
            planner.updateMap(maplayer);
            planner.replan(plan, arm_current);
            printf("Total length %ld\n",plan.size());
            t_endplan = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> t_plandiff = t_endplan - t_startplan;
            double t_plan = t_plandiff.count()/1000.0;

            printf("replanning took %f seconds\n", t_plan);

            //Increment t by t_plan & update arm_traj to stay in place at the skipped times:
            for(int t_wait = t; t_wait < (t+=(int) floor(t_plan)); t_wait++){
                traj_vector.push_back(arm_current);
            }
            t += (int) floor(t_plan);

            //reset arm_next and next_plan_step:
            arm_next = arm_current;
            next_plan_step = 1;
        }
        else{
            printf("MOVING ARM\n");
            bool next_plan_reached = increment_arm(arm_next, arm_current, maxjntspeed, plan[next_plan_step], numofDOFs);
            printf("INCREMENT ARM DONE\n");
            arm_current = arm_next;

            //insert into arm_traj
            traj_vector.push_back(arm_current);

            //increment plan step if reached
            if(next_plan_reached){
                next_plan_step++;
            }
        }
        if (arm_goal == arm_current)
        {
            printf("reached GOAL !!!!!!!!!!\n");
            break;
        }
    }

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span = t2 - t1;
    double time = time_span.count()/1000.0;

    printf("Planner Took: %f seconds\n",time);

    /* Convert arm trajectory to return to mex */
    double** arm_traj = NULL;
    int traj_length = traj_vector.size();
    printf("traj length : %d\n",traj_length);
    if(traj_length > 0){
        arm_traj = (double**) malloc(traj_length*sizeof(double*));
        for (int i = 0; i < traj_length; i++){
            arm_traj[i] = (double*) malloc(numofDOFs*sizeof(double)); 
            for(int j = 0; j < numofDOFs; j++){
                arm_traj[i][j] = traj_vector[i][j];
            }
        }
    }

    /* Create return values */
    if(traj_length > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)traj_length, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < traj_length; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*traj_length + i] = arm_traj[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }     
    }
    int num_vertices = 0;
    double cost = 0;
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL);
    PATHTIME_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL); 
    PATHCOST_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL);
    NUMVERTICES_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = traj_length;
    double* plantime_out = (double*) mxGetPr(PATHTIME_OUT);
    *plantime_out = time;
    double* plancost_out = (double*) mxGetPr(PATHCOST_OUT);
    *plancost_out = cost;
    double* planvertices_out = (double*) mxGetPr(NUMVERTICES_OUT);
    *planvertices_out = num_vertices;
    return;
}