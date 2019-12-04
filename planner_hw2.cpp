// #include <RRT.h>
// #include <RRTConnect.h>
#include <RRTStar.h>
#include <PRM.h>

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT_        0
#define RRTCONNECT_ 1
#define RRTSTAR_    2
#define PRM_        3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]
#define	PLANCOST_OUT	plhs[2]
#define	PLANTIME_OUT	plhs[3]
#define	VERTEX_OUT	plhs[4]
#define	UNDER_5S_OUT	plhs[5]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

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
    } else if (nlhs != 6) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
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
 
    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");         
    }
    //call the planner
    vector<vector<double>> plan{};
    int planlength = 0;
    vector<double> armstart_anglesV_rad_vec(armstart_anglesV_rad,armstart_anglesV_rad + numofDOFs);
    vector<double> armgoal_anglesV_rad_vec(armgoal_anglesV_rad,armgoal_anglesV_rad + numofDOFs);
    double time_,cost,s5;
    int vertices;
    //you can may be call the corresponding planner function here
    if (planner_id == RRT_)
    {
      RRT *planner = new RRT(map, x_size, y_size, armstart_anglesV_rad_vec, armgoal_anglesV_rad_vec, numofDOFs);
      plan = planner->plan();
      planlength = plan.size();
      time_ = planner->get_time();
      cost = planner->get_cost();
      s5 = planner->get_s5();
      vertices = planner->get_vertices();
      delete planner;
    }
    else if (planner_id == RRTCONNECT_)
    {
      RRTCONNECT *planner = new RRTCONNECT(map, x_size, y_size, armstart_anglesV_rad_vec, armgoal_anglesV_rad_vec, numofDOFs);
      plan = planner->plan();
      planlength = plan.size();
      time_ = planner->get_time();
      cost = planner->get_cost();
      s5 = planner->get_s5();
      vertices = planner->get_vertices();
      delete planner;
    }    
    else if (planner_id == RRTSTAR_)
    {
      RRTSTAR *planner = new RRTSTAR(map, x_size, y_size, armstart_anglesV_rad_vec, armgoal_anglesV_rad_vec, numofDOFs);
      plan = planner->plan();
      planlength = plan.size();
      time_ = planner->get_time();
      cost = planner->get_cost();
      s5 = planner->get_s5();
      vertices = planner->get_vertices();
      delete planner;
    }
    else if (planner_id == PRM_)
    {
      PRM *planner = new PRM(map, x_size, y_size, armstart_anglesV_rad_vec, armgoal_anglesV_rad_vec, numofDOFs);
      plan = planner->plan();
      planlength = plan.size();
      time_ = planner->get_time();
      cost = planner->get_cost();
      s5 = planner->get_s5();
      vertices = planner->get_vertices();
      delete planner;
    }  

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
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;

    PLANCOST_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL); 
    double* plancost_out = (double*) mxGetPr(PLANCOST_OUT);
    *plancost_out = cost;
    PLANTIME_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL); 
    double* plantime_out = (double*) mxGetPr(PLANTIME_OUT);
    *plantime_out = time_;
    VERTEX_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT16_CLASS, mxREAL); 
    int* vertex_out = (int*) mxGetPr(VERTEX_OUT);
    *vertex_out = vertices;
    UNDER_5S_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL); 
    double* under_5s_out = (double*) mxGetPr(UNDER_5S_OUT);
    *under_5s_out = s5;
    return;
}