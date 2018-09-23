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
#include <chrono>
#include <time.h>
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
static std::map<std::pair<int,int>, int> heuristic_map;
static std::map<std::pair<int,int>, Node> a_star_closed_map;
static int path_idx = 0;
static std::vector<Node> path;
time_t start_time;

struct cmpNode {
    bool operator()(const Node& a, const Node& b) const {
        return a.value > b.value;
    }
};

static std::priority_queue<Node, std::vector<Node>, cmpNode> prior_q;

void save_djikstra(int x_size, int y_size)
{
    std::cout << "saving" << std::endl;
    std::ofstream myfile;
    myfile.open ("example.txt");
    // for (int i = 0; i < x_size; i++)
    // {
    //     for (int j = 0; j < y_size; j++)
    //     {
    //         char buffer[6];
    //         int n;
    //         if (heuristic_map[GETMAPINDEX(i,j,x_size,y_size)] > 100000)
    //             n = sprintf(buffer, "%d", 0);
    //         else n = sprintf(buffer, "%d", heuristic_map[GETMAPINDEX(i,j,x_size,y_size)]);
    //         myfile << buffer;
    //         if (j < y_size-1) myfile << ",";
    //         else myfile << "\n";
    //     }
    // }
    myfile.close();
    return;
}

void djikstra(double *map, int x_size, int y_size, int target_steps, double* target_traj, int collision_thresh)
{
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    std::priority_queue<Node, std::vector<Node>, cmpNode> pq;
    std::pair<std::map<std::pair<int,int>, int>::iterator, bool> result;
    for (int i = 0; i < x_size; i++)
    {
        for (int j = 0; j < y_size; j++)
        {
            result = heuristic_map.insert(std::pair<std::pair<int,int>, int>(std::make_pair(i,j), INT_MAX));
        }
    }
    for (int i = 0; i < target_steps; i++)
    {
        heuristic_map[std::make_pair((int) target_traj[i], (int) target_traj[i + target_steps])] = 0;
        int cost = 0;
        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = (int) target_traj[i] + dX[dir];
            int newy = (int) target_traj[i + target_steps] + dY[dir];
            if ((newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) &&
                ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) &&
                ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh) &&
                (heuristic_map.at(std::make_pair(newx,newy)) > map[GETMAPINDEX((int) target_traj[i], (int) target_traj[i + target_steps], x_size, y_size)] + cost))
            {
                Node new_node;
                new_node.x = newx;
                new_node.y = newy;
                new_node.value = map[GETMAPINDEX(newx,newy,x_size,y_size)] + cost;
                pq.push(new_node);
            }
        }
    }
    // std::cout << "running djikstra" << std::endl;
    int count = 0;
    while (!pq.empty())
    {
        count++;
        bool bad_node = true;
        Node curr_node;
        do {
            curr_node = pq.top();
            pq.pop();
            if (heuristic_map.count(std::make_pair(curr_node.x, curr_node.y)) > 0 &&
                heuristic_map.at(std::make_pair(curr_node.x, curr_node.y)) > curr_node.value +
                map[GETMAPINDEX(curr_node.x, curr_node.y, x_size, y_size)])
                bad_node = false;
        } while (bad_node && !pq.empty());
        heuristic_map[std::make_pair(curr_node.x, curr_node.y)] = curr_node.value;
        int cost = curr_node.value;
        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = curr_node.x + dX[dir];
            int newy = curr_node.y + dY[dir];
            if ((newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) &&
                ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) &&
                ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh) &&
                heuristic_map.count(std::make_pair(newx,newy)) > 0 &&
                (heuristic_map.at(std::make_pair(newx,newy)) > map[GETMAPINDEX(curr_node.x, curr_node.y, x_size, y_size)] + cost))
            {
                Node new_node;
                new_node.x = newx;
                new_node.y = newy;
                new_node.value = map[GETMAPINDEX(newx,newy,x_size,y_size)] + cost;
                pq.push(new_node);
            }
        }
    }
    return;
}

void extract_path(int x, int y, int robotposeX, int robotposeY, int x_size, int y_size)
{
    Node curr_node;
    curr_node = a_star_closed_map.at(std::make_pair(x,y));
    while ((double)sqrt(((curr_node.x-robotposeX)*(curr_node.x-robotposeX) + (curr_node.y-robotposeY)*(curr_node.y-robotposeY))) > 0.5)
    {
        // std::cout << "curr_node = " << curr_node.x << ", " << curr_node.y << ", " << curr_node.value << std::endl;
        path.push_back(curr_node);
        curr_node = a_star_closed_map.at(std::make_pair(curr_node.parent_x,curr_node.parent_y));
    }
    // std::cout << "path size = " << (int) path.size() << std::endl;
    path_idx = ((int) path.size()) - 1;
    // std::cout << "path idx = " << path_idx << std::endl;
    return;
}

bool is_finished(int x, int y, int plan_time, int curr_time, int target_steps, const double* target_traj)
{
    int time_diff = difftime(time(0), start_time);
    if (target_steps -1 > curr_time + plan_time + time_diff)
    {
        int target_x = target_traj[curr_time+time_diff+plan_time];
        int target_y = target_traj[target_steps+time_diff+plan_time+curr_time];
        // std::cout << "end condition = " << (double) sqrt(((x-target_x)*(x-target_x) + (y-target_y)*(y-target_y))) << std::endl;
        // std::cout << "goal pose = " << target_x << ", " << target_y << std::endl;

        if ((double) sqrt(((x-target_x)*(x-target_x) + (y-target_y)*(y-target_y))) < 0.5)// &&
            // curr_time == (target_steps + time_diff))
            return true;
        else return false;
    } else
    {
        // std::cout << "too slow" << std::endl;
      return false;  
    } 
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
    start_time = time(0);
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::seconds>(now);
    
    auto value = now_ms.time_since_epoch();
    double seconds_since_start = difftime( time(0), start_time);
    int duration = value.count();
    int intial_dur = value.count();
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    int dirx = robotposeX;
    int diry = robotposeY;

    int tempx = robotposeX;
    int tempy = robotposeY;
    duration = value.count();
    // std::cout << "duration = " << duration - intial_dur << std::endl;
    // Node node = Node(tempx, tempy, ((int)map[GETMAPINDEX(tempx,tempy,x_size,y_size)]), robotposeX, robotposeY,
        // ((int)map[GETMAPINDEX(tempx,tempy,x_size,y_size)]), 0);
    Node node;
    node.x = tempx;
    node.y = tempy;
    node.value = ((int)map[GETMAPINDEX(tempx,tempy,x_size,y_size)]);
    node.parent_x = robotposeX;
    node.parent_y = robotposeY;
    node.g_value = ((int)map[GETMAPINDEX(tempx,tempy,x_size,y_size)]);
    node.time = 0;
    // node.visited = 1;
    // node.time = curr_time;
    // std::cout << "running " << std::endl;
    // std::cout << "heuristic_generated = " << heuristic_generated << std::endl;
    if (!heuristic_generated)
    {
        std::cout << "djikstra" << std::endl;
        djikstra(map, x_size, y_size, target_steps, target_traj, collision_thresh);
        heuristic_generated = true;
        prior_q.push(node);
        duration = value.count();
        // seconds_since_start = difftime( time(0), start);
        // std::cout << "seconds_since_start = " << seconds_since_start << std::endl;
        // std::cout << "duration = " << duration - intial_dur << std::endl;
    }

    // std::cout << "planning " << std::endl;
    int cost = heuristic_map.at(std::make_pair(robotposeX, robotposeY));
    int iter_count = 0;
    std::pair<std::map<std::pair<int,int>, Node>::iterator, bool> result;
    if (!path_found)
    {
        std::cout << "planning " << std::endl;
        while (!path_found && (double) sqrt(((tempx-goalposeX)*(tempx-goalposeX) + (tempy-goalposeY)*(tempy-goalposeY))) > 0.5)
        // while (iter_count < 600 && !path_found && (double) sqrt(((tempx-goalposeX)*(tempx-goalposeX) + (tempy-goalposeY)*(tempy-goalposeY))) > 0.5)
        {
            duration = value.count();
            // seconds_since_start = difftime( time(0), start);
            // std::cout << "seconds_since_start = " << seconds_since_start << std::endl;
            // std::cout << "duration = " << duration - intial_dur << std::endl;
            iter_count++;
            // std::cout << iter_count << std::endl;
            bool bad_node = true;
            Node curr_node;
            do {
                curr_node = prior_q.top();
                prior_q.pop();
                // if (a_star_closed_map.count(GETMAPINDEX(curr_node.x, curr_node.y, x_size, y_size)) == 0)
                if (a_star_closed_map.count(std::make_pair(curr_node.x, curr_node.y)) == 0)
                {
                    bad_node = false;
                    break;
                }
            } while (bad_node && !prior_q.empty());
            // std::pair<std::map<int, Node>::iterator, bool> result;
            // result = a_star_closed_map.insert(std::pair<int, Node>(GETMAPINDEX(curr_node.x,curr_node.y,x_size,y_size), curr_node));
            result = a_star_closed_map.insert(std::pair<std::pair<int,int>, Node>(std::make_pair(curr_node.x,curr_node.y), curr_node));
            for (int dir = 0; dir < NUMOFDIRS; dir++)
            {
                int newx = curr_node.x + dX[dir];
                int newy = curr_node.y + dY[dir];
                if ((newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) &&
                    ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) &&
                    ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh) &&
                    // a_star_closed_map.count(std::make_pair(newx,newy)) == 0 &&
                    curr_node.time < target_steps)
                {
                    Node new_node;
                    new_node.x = newx;
                    new_node.y = newy;
                    new_node.parent_x = curr_node.x;
                    new_node.parent_y = curr_node.y;
                    new_node.g_value = ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)]) + curr_node.g_value;
                    int h;
                    if (heuristic_map.count(std::make_pair(newx,newy)) > 0)
                    {
                        int h1 = heuristic_map.at(std::make_pair(newx,newy));
                        int time_diff = difftime(time(0), start_time);
                        int target_x = target_traj[curr_time+time_diff+curr_node.time + 1];
                        int target_y = target_traj[target_steps+time_diff+curr_node.time + 1 +curr_time];
                        int h2 = (double)sqrt(((newx-target_x)*(newx-target_x) + (newy-target_y)*(newy-target_y)));
                        // int h2 = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                        if (1.5*h1 >= h2)
                        // if (h1 >= h2)
                        // if (true)
                        {
                            h = h1;
                            // std::cout << "h1" << std::endl;
                        }
                        else 
                        {
                            h = h2;
                            // std::cout << "h2" << std::endl;
                        }
                    }
                    else h = INT_MAX;
                    new_node.value = new_node.g_value + 10 * h;
                    new_node.time = curr_node.time + 1;

                    if (a_star_closed_map.count(std::make_pair(newx,newy)) == 0)
                    {
                        prior_q.push(new_node);    
                    } else
                    {
                        Node next_node;
                        next_node = a_star_closed_map.at(std::make_pair(newx,newy));
                        if (next_node.time != new_node.time) prior_q.push(new_node);
                    }
                    
                    // std::cout << "new node = " << newx << ", " << newy << ", " << curr_node.time+1 << ", " << new_node.value << std::endl;
                }
            }
            tempx = curr_node.x;
            tempy = curr_node.y;
            int time_diff = difftime(time(0), start_time);
            // std::cout << "curr pose = " << tempx << ", " << tempy << ", " << curr_node.time << std::endl;
            // // std::cout << "goal pose = " << goalposeX << ", " << goalposeY << std::endl;
            // // std::cout << "end condition = " << (double) sqrt(((tempx-goalposeX)*(tempx-goalposeX) + (tempy-goalposeY)*(tempy-goalposeY))) << std::endl;
            // std::cout << "time step = " << curr_node.time + time_diff << std::endl;
            // std::cout << "robot time = " << curr_time + time_diff << std::endl;
            // std::cout << "pq.size() = " << (int) prior_q.size() << std::endl;
            // if ((double)sqrt(((tempx-goalposeX)*(tempx-goalposeX) + (tempy-goalposeY)*(tempy-goalposeY))) < 0.5 || path_found)
            // if (collided())
            if (is_finished(tempx, tempy, curr_node.time, curr_time, target_steps, target_traj) || path_found)
            // if ((double)  sqrt(((tempx-goalposeX)*(tempx-goalposeX) + (tempy-goalposeY)*(tempy-goalposeY))) < 0.5 || path_found)
            {

                extract_path(tempx, tempy, robotposeX, robotposeY, x_size, y_size);
                path_found = true;
                printf("goal: %d %d;\n", goalposeX, goalposeY);
            }
        }

    }
    // std::cout << "path found = " << path_found << std::endl;
    if (path_found && path_idx > 0)
    {
        Node next_node;
        next_node = path[path_idx];
        path_idx--;
        // std::cout << "curr pose = " << robotposeX << ", " << robotposeY << std::endl;
        // std::cout << "next pose = " << next_node.x << ", " << next_node.y << std::endl;
        action_ptr[0] = next_node.x;
        action_ptr[1] = next_node.y;
    }
    else
    {
        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = robotposeX + dX[dir];
            int newy = robotposeY + dY[dir];
            if ((newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) &&
                ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) &&
                ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh)) 
            {
                if (heuristic_map.count(std::make_pair(newx,newy)) > 0 &&
                    heuristic_map.at(std::make_pair(newx,newy)) < cost)
                {
                    cost = heuristic_map.at(std::make_pair(newx,newy));
                    dirx = newx;
                    diry = newy;
                }
            }
        }
        // std::cout << "dir = " << dirx << ", " << diry << std::endl;
        action_ptr[0] = dirx;
        action_ptr[1] = diry;
    }
    // std::cout << "broke for some reason" << std::endl;
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
    // std::cout << "done planning" << std::endl;
    return;   
}