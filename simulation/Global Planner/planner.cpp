#include <iostream>
#include <string>

using namespace std;

#include <sbpl/headers.h>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/heap.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/list.h>
#include <sbpl/headers.h>
#include <sbpl/heuristics/embedded_heuristic.h>
#include <sbpl/planners/mhaplanner.h> 

#define DEBUG 1


void initializeEnv( EnvironmentNAVXYTHETALAT& env, vector<sbpl_2Dpt_t>& perimeter, char* envCfgFilename, char* motPrimFilename)

{
	if (!env.InitializeEnv(envCfgFilename, perimeter, motPrimFilename))
		{
        printf("ERROR: InitializeEnv failed\n");
        throw SBPL_Exception();
		}
}

void createFootprint(vector<sbpl_2Dpt_t>& perimeter, double length, double width)
{
    sbpl_2Dpt_t pt_m;
    pt_m.x = -(length/2);
    pt_m.y = -(width/2);
    perimeter.push_back(pt_m);
    pt_m.x = (length/2);
    pt_m.y = -(width/2);
    perimeter.push_back(pt_m);
    pt_m.x = (length/2);
    pt_m.y = (width/2);
    perimeter.push_back(pt_m);
    pt_m.x = -(length/2);
    pt_m.y = (width/2);
    perimeter.push_back(pt_m);
}

void setEnvStartGoal(EnvironmentNAVXYTHETALAT& env, 
                     double start_x, double start_y, double start_theta,
                     double goal_x, double goal_y, double goal_theta, 
                     int& start_id, int& goal_id)
{
    start_id = env.SetStart(start_x, start_y, start_theta); 
    goal_id = env.SetGoal(goal_x, goal_y, goal_theta);
}

void initializePlanner(SBPLPlanner*& planner, 
                       EnvironmentNAVXYTHETALAT& env,
                       int start_id, int goal_id,
                       double initialEpsilon, 
                       bool bsearchuntilfirstsolution)
{
    
    bool bsearch = false;   
    planner = new ARAPlanner(&env,bsearchuntilfirstsolution);// intialise correctly
    // set planner properties
    if (planner->set_start(start_id) == 0) {
        printf("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    }
    if (planner->set_goal(goal_id) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    }
    planner->set_initialsolution_eps(initialEpsilon);
    planner->set_search_mode(bsearchuntilfirstsolution);
}

int runPlanner(SBPLPlanner* planner, int allocated_time_secs, 
               vector<int>&solution_stateIDs){
    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs);
 
    if (bRet) 
        printf("Solution is found\n");
    else 
        printf("Solution does not exist\n");
    return bRet;
}

//to explicitly write solutions to a file
void writeSolution(EnvironmentNAVXYTHETALAT& env, vector<int> solution_stateIDs, const char* filename)
{
    std::string discrete_filename(std::string(filename) + std::string(".discrete"));
    FILE* fSol_discrete = fopen(discrete_filename.c_str(), "w");
    FILE* fSol = fopen(filename, "w");
    if (fSol == NULL) {
        printf("ERROR: could not open solution file\n");
        throw SBPL_Exception();
    }
 
    // write the discrete solution to file
    for (size_t i = 0; i < solution_stateIDs.size(); i++) {
        int x, y, theta;
        env.GetCoordFromState(solution_stateIDs[i], x, y, theta);
        double cont_x, cont_y, cont_theta;
        cont_x = DISCXY2CONT(x, 0.1);
        cont_y = DISCXY2CONT(y, 0.1);
        cont_theta = DiscTheta2Cont(theta, 16);
        fprintf(fSol_discrete, "%d %d %d\n", x, y, theta);
    }
    fclose(fSol_discrete);
 
    // write the continuous solution to file
    vector<sbpl_xy_theta_pt_t> xythetaPath;
    env.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &xythetaPath);
    for (unsigned int i = 0; i < xythetaPath.size(); i++) {
        fprintf(fSol, "%.3f %.3f %.3f\n", xythetaPath.at(i).x, 
                                          xythetaPath.at(i).y, 
                                          xythetaPath.at(i).theta);
    }
    fclose(fSol);
}

void planxythetalat(char* envCfgFilename, char* motPrimFilename, float start_x,float start_y, float goal_x,float goal_y)

{

    double l1=0.01,w1=0.01,l2=0.02,w2=0.02; // set the perimeter of the robot   
    vector<sbpl_2Dpt_t> perimeter;
    createFootprint(perimeter,l1,w1); //Passed perimeter by reference 

    EnvironmentNAVXYTHETALAT env; // initialize an environment  
    initializeEnv(env, perimeter1,envCfgFilename, motPrimFilename);
    
    // specify a start and goal state
    int start_id, goal_id;

    setEnvStartGoal(env1, start_x, start_y, 0, goal_x, goal_y, 0, start_id, goal_id);
    

    // initialize a planner with start and goal state
    SBPLPlanner* planner = NULL;
    double initialEpsilon = 5.0;
    bool bsearchuntilfirstsolution = false;
    initializePlanner(planner, env, start_id, goal_id, initialEpsilon, 
                      bsearchuntilfirstsolution);
    
    // plan
    vector<int> solution_stateIDs;
    double allocated_time_secs = 2.0; // in seconds
    runPlanner(planner, allocated_time_secs, solution_stateIDs);
 
    // print stats
    env1.PrintTimeStat(stdout);

if DEBUG
{
    double trun;
    trun= planner->get_initial_eps_planning_time();
    cout<<"Time to Run: "<<trun;

    int expansions;
    expansions= planner->get_n_expands();
    cout<<endl<<"Expansions: "<<expansions<<endl;
 }
    // write out solutions
    std::string filename("sol.txt");
    writeSolution(env, solution_stateIDs, filename.c_str());
 
    delete planner;
}

int main(int argc, char *argv[])
{
    //planxythetalat(argv[1], argv[2],argv[3],argv[4],argv[5],argv[6]);
	planxythetalat(argv[1], argv[2],0.11,0.11, 0.35, 0.3);
}