/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <queue>
#include <map>
#include <iostream>
#include <fstream>
#include "node.h"

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8


static bool heuristic_generated = false;
static bool path_found = false;
static int* heuristic_map = NULL;

struct cmpNode {
    bool operator()(const Node& a, const Node& b) const {
        return a.value > b.value;
    }
};

void save_djikstra(int x_size, int y_size)
{
    std::cout << "saving" << std::endl;
    std::ofstream myfile;
    myfile.open ("example.txt");
    for (int i = 0; i < x_size; i++)
    {
        for (int j = 0; j < y_size; j++)
        {
            char buffer[6];
            int n;
            if (heuristic_map[GETMAPINDEX(i,j,x_size,y_size)] > 100000)
                n = sprintf(buffer, "%d", 0);
            else n = sprintf(buffer, "%d", heuristic_map[GETMAPINDEX(i,j,x_size,y_size)]);
            myfile << buffer;
            if (j < y_size-1) myfile << ",";
            else myfile << "\n";
        }
    }
    myfile.close();
    return;
}

void djikstra(double *map, int x_size, int y_size, int target_steps, double* target_traj, int collision_thresh)
{
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    heuristic_map = new int[x_size*y_size];
    std::priority_queue<Node, std::vector<Node>, cmpNode> pq;

    for (int i = 0; i < x_size; i++)
    {
        for (int j = 0; j < y_size; j++)
            heuristic_map[GETMAPINDEX(i,j,x_size,y_size)] = INT_MAX;
    }
    for (int i = 0; i < target_steps; i++)
    {
        heuristic_map[GETMAPINDEX((int) target_traj[i], (int) target_traj[i + target_steps], x_size, y_size)] = 0;
        int cost = 0;
        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = (int) target_traj[i] + dX[dir];
            int newy = (int) target_traj[i + target_steps] + dY[dir];
            if ((newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) &&
                ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) &&
                ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh) &&
                heuristic_map[GETMAPINDEX(newx, newy, x_size, y_size)] > map[GETMAPINDEX((int) target_traj[i], (int) target_traj[i + target_steps], x_size, y_size)] + cost)
            {
                Node new_node;
                new_node.x = newx;
                new_node.y = newy;
                new_node.value = map[GETMAPINDEX(newx,newy,x_size,y_size)] + cost;
                pq.push(new_node);
            }
        }
    }
    std::cout << "running djikstra" << std::endl;
    int count = 0;
    while (!pq.empty())
    {
        count++;
        bool bad_node = true;
        Node curr_node;
        do {
            curr_node = pq.top();
            pq.pop();
            if (heuristic_map[GETMAPINDEX(curr_node.x, curr_node.y, x_size, y_size)] > curr_node.value +
                map[GETMAPINDEX(curr_node.x, curr_node.y, x_size, y_size)])
                bad_node = false;
        } while (bad_node && !pq.empty());
        heuristic_map[GETMAPINDEX(curr_node.x, curr_node.y, x_size, y_size)] = curr_node.value;
        int cost = curr_node.value;
        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = curr_node.x + dX[dir];
            int newy = curr_node.y + dY[dir];
            if ((newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) &&
                ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) &&
                ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh) &&
                heuristic_map[GETMAPINDEX(newx, newy, x_size, y_size)] > map[GETMAPINDEX(curr_node.x, curr_node.y, x_size, y_size)] + cost)
            {
                Node new_node;
                new_node.x = newx;
                new_node.y = newy;
                new_node.value = map[GETMAPINDEX(newx,newy,x_size,y_size)] + cost;
                pq.push(new_node);
            }
        }
    }
    save_djikstra(x_size, y_size);
    return;
}

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    double disttotarget;
    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
            {
                disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                if(disttotarget < olddisttotarget)
                {
                    olddisttotarget = disttotarget;
                    bestX = dX[dir];
                    bestY = dY[dir];
                }
            }
        }
    }
    robotposeX = robotposeX + bestX;
    robotposeY = robotposeY + bestY;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    std::cout << "djikstra" << std::endl;
    djikstra(map, x_size, y_size, target_steps, target_traj, collision_thresh);
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}