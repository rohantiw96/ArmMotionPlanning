#include <math.h>
#include "mex.h"
#include <time.h>
// #include "include/sampling_planner.h"
// #include "LazyPRM/lazy_prm.h"
#include <iostream>
#include <vector>
#include <chrono>


/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

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

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    /* Check for proper number of arguments */    
    if (nrhs != 4) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required."); 
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
    
    //call the planner
    double** plan = NULL;
    int planlength = 0;

    //Tunable parameters
    int lookahead = 3;
    double maxjntspeed = 0.5;
    
    //you can may be call the corresponding planner function here
    double cost = 0;
    double num_vertices = 0;
    int num_iterations = 100000;  
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    printf("0: %f, 2500: %f, 5000: %f\n", map[0], map[2500], map[5000]);
    
    double* maplayer = &map[2500];
    printf("0: %f, 2500: %f, 5000: %f\n", maplayer[0], maplayer[2500], maplayer[5000]);

    printf("map xdim: %d, ydim: %d, tdim: %d\n", x_size, y_size, t_size);

    // Call Planner Here Exmaple: 

    // SamplingPlanner planner(map,x_size,y_size,arm_start,arm_goal,numofDOFs,epsilon,samples,num_iterations);
    // planner.plan(&plan, &planlength);
    
    // LAZYPRM planner(map,x_size,y_size,arm_start,arm_goal,numofDOFs);
    // planner.plan(&plan, &planlength);
    // cost = planner.returnPathCost();
    // num_vertices = planner.returnNumberOfVertices();

    int layersize = x_size * y_size;
    std::vector<double> arm_current = arm_start;
    int next_plan_step = 1;
    for(int t=0; t < t_size; t++){
        int layer_index = layersize * t;
        maplayer = &map[layer_index];

        if(t==0){
            std::chrono::high_resolution_clock::time_point t_startplan = std::chrono::high_resolution_clock::now();
            planner.generate(maplayer, x_size, y_size, arm_start, arm_goal, numofDOFs);
            std::chrono::high_resolution_clock::time_point t_endplan = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> t_plandiff = t_endplan - t_startplan;
            double t_plan = t_plandiff.count()/1000.0;

            //Increment t by t_plan:
            t += floor(t_plan);
        }


    }

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span = t2 - t1;
    double time = time_span.count()/1000.0;

    printf("Planner Took: %f seconds\n",time);
    // printf("Planner returned plan of length=%d\n", planlength); 
    printf("Total Cost %f\n",cost);
    printf("Number of Vertices %f\n",num_vertices);
    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
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
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL);
    PATHTIME_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL); 
    PATHCOST_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL);
    NUMVERTICES_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL); 
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;
    double* plantime_out = (double*) mxGetPr(PATHTIME_OUT);
    *plantime_out = time;
    double* plancost_out = (double*) mxGetPr(PATHCOST_OUT);
    *plancost_out = cost;
    double* planvertices_out = (double*) mxGetPr(NUMVERTICES_OUT);
    *planvertices_out = num_vertices;
    return;
    
}