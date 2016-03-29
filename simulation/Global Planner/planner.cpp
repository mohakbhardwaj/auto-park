#include <iostream>
#include <string>
#include <fstream>
#include <iterator>
#include <sstream>
#include <vector>



using namespace std;

#include <sbpl/headers.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/heap.h>
#include <sbpl/utils/key.h>
#include <sbpl/utils/list.h>
#include <sbpl/headers.h>
#include <sbpl/heuristics/embedded_heuristic.h>


#include <sbpl/planners/mhaplanner.h> //doesn't work without the "" 

#define DEBUG_P (1)

#define COMPARE (0)

struct envState{

float x; //x location of the state in meters
float y; //y location of that state in meters
float th; //theta at that state

};




//costmap2d

//generate multiple heuristics and call sbpl planner for mha*

//initialise the environment, throw an exception if it doesn't initialise

void initializeEnv( EnvironmentNAVXYTHETALAT& env, vector<sbpl_2Dpt_t>& perimeter1, //vector<sbpl_2Dpt_t>& perimeter2,
                    char* envCfgFilename, char* motPrimFilename)

{
    if (!env.InitializeEnv(envCfgFilename, perimeter1, //perimeter2, 
                        motPrimFilename))
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

//set a start goal- starting and finishing x,y and theta
//also mention a start ID and goal id for the lattice
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
                       bool bsearchuntilfirstsolution,Heuristic* hanchor, Heuristic* heurs,int hcount){
    
    bool bsearch = false;
    MHAPlanner *mha_planner = new MHAPlanner(&env,hanchor,&heurs,hcount);// intialise correctly
    mha_planner->set_initial_mha_eps(5);

    planner = mha_planner;

    //planner = new ARAPlanner(&env,bsearchuntilfirstsolution);// intialise correctly
 
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
    //planner->set_initial_mha_eps(3);
    planner->set_search_mode(bsearchuntilfirstsolution);

}


void initializePlanner(SBPLPlanner*& planner, 
                       EnvironmentNAVXYTHETALAT& env,
                       int start_id, int goal_id,
                       double initialEpsilon, 
                       bool bsearchuntilfirstsolution){
    // work this out later, what is bforwardsearch?
    
    bool bsearch = false;
    
    
    planner = new ARAPlanner(&env, bsearch);
    
    
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
 
    if (bRet && DEBUG_P) 
        printf("Solution is found\n");
    else 
        printf("Solution does not exist\n");
    return bRet;
}
 
void writeSolution(EnvironmentNAVXYTHETALAT& env, vector<int> solution_stateIDs,
                   const char* filename){
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

void populateGoals( std::vector<envState>& start, std::vector<envState>& goal)
 {  
    envState pt1,pt2;
    pt1.th=0;
    pt2.th=0;

    pt1.x=10.25;
    pt1.y=17.25;
    pt2.x=46.0;
    pt2.y=54.0;
    /*
    pt1.x=.11;
    pt1.y=.11;
    pt2.x=.35;
    pt2.y=.3;
    */

    start.push_back(pt1);
    goal.push_back(pt2);

    
 }

 void populateRandomGoals (std::vector<envState>& start, std::vector<envState>& goal)
{
    //double width=48.675;
    //double height=55.275;
    
    envState pt1,pt2;
    pt1.th=pt2.th=0;

    pt1.x=10.25;
    pt1.y=17.25;
    pt2.x=46.0;
    pt2.y=54.0;

    std::default_random_engine generator;  //generate a normal distribution around default set of goals of the environment
    std::normal_distribution<double> pt1x(pt1.x,2.0);
    std::normal_distribution<double> pt1y(pt1.y,2.0);
    std::normal_distribution<double> pt2x(pt2.x,2.0);
    std::normal_distribution<double> pt2y(pt2.y,2.0);
    
    for(int i = 0; i <9; i++)
    {
       pt1.x=roundf(pt1x(generator)*100)/100;
       pt1.y=roundf(pt1y(generator)*100)/100;
       pt2.x=roundf(pt2x(generator)*100)/100;
       pt2.y=roundf(pt2y(generator)*100)/100;
       start.push_back(pt1);
       goal.push_back(pt2);


    }

        //env.GetCoordFromState(int stateID, int& x, int& y, int& theta) const;

}


void planxythetalat(char* envCfgFilename, char* motPrimFilename)

{
    double l1=0.01,w1=0.01,l2=0.02,w2=0.02;
    // set the perimeter of the robot   
    vector<sbpl_2Dpt_t> perimeter1, perimeter2;
    createFootprint(perimeter1,l1,w1); //Passed perimeter by reference
    // createFootprint(perimeter2,l2,w2); //Passed perimeter by reference


    
    // specify a start and goal state
    int start_id, goal_id;
    
    std::vector<envState> start,goal;

    //TODO: Populate start and goal states
    
    //populateGoals(start,goal); //populate a single goal
    populateRandomGoals(start,goal);


   
    SBPLPlanner* planner = NULL;
    double initialEpsilon = 5.0;
    bool bsearchuntilfirstsolution = false;
    vector<int> solution_stateIDs1,solution_stateIDs2 ;
    double allocated_time_secs = 5.0; // in seconds
    
    std::vector<pair<double,double> > trun,pathSize,expansions,eps;
    double MHAtime,MHAexpands,MHAlength,MHAeps;
    double ARAtime,ARAexpands,ARAlength,ARAeps;

  for (int i = 0; i<start.size(); i++)
    {       
        EnvironmentNAVXYTHETALAT env1;
        EnvironmentNAVXYTHETALAT env2;

         Heuristic* hanchor = new EmbeddedHeuristic(&env1);
    Heuristic* h1 = new EmbeddedHeuristic(&env1);

    
        initializeEnv(env1, perimeter1,envCfgFilename,motPrimFilename);
        initializeEnv(env2, perimeter1,envCfgFilename,motPrimFilename);
        SBPLPlanner* MHAplanner = NULL;
        
        SBPLPlanner* ARAplanner=NULL ;
        setEnvStartGoal(env1, start[i].x, start[i].y, start[i].th, goal[i].x, goal[i].y, goal[i].th, 
                            start_id, goal_id);
        setEnvStartGoal(env2, start[i].x, start[i].y, start[i].th, goal[i].x, goal[i].y, goal[i].th, 
                            start_id, goal_id);

        
        //-----------------MHA-----------------//
        cout<<endl<<"MHA goal:"<<i<<endl;
        double initialEpsilon = 5.0;
        bool bsearchuntilfirstsolution = false;
        vector<int> solution_stateIDs1,solution_stateIDs2 ;

 
        initializePlanner(MHAplanner, env1, start_id, goal_id, initialEpsilon, 
                      bsearchuntilfirstsolution, hanchor,h1,1);
        runPlanner(MHAplanner, allocated_time_secs, solution_stateIDs1);
    
        std::string filename1("MHAsol"+std::to_string(i)+".txt"); // write out solutions
        writeSolution(env1, solution_stateIDs1, filename1.c_str());

        MHAtime=MHAplanner->get_initial_eps_planning_time();
        MHAexpands=MHAplanner->get_n_expands_init_solution();
        MHAlength=solution_stateIDs1.size();
        MHAeps=MHAplanner->get_final_epsilon();
        delete MHAplanner;


        if COMPARE 
        {

        //-----------------ARA-----------------//
            cout<<"ARA goal:"<<i<<endl;
            initialEpsilon = 5.0;
            bsearchuntilfirstsolution = false;
            
            
            initializePlanner(ARAplanner, env2, start_id, goal_id, initialEpsilon, 
                          bsearchuntilfirstsolution);

            runPlanner(ARAplanner, allocated_time_secs, solution_stateIDs2);
            std::string filename2("ARAsol"+std::to_string(i)+".txt"); // write out solutions
            writeSolution(env2, solution_stateIDs2, filename2.c_str());
            
            ARAtime=ARAplanner->get_initial_eps_planning_time();
            ARAexpands=ARAplanner->get_n_expands_init_solution();
            ARAlength=solution_stateIDs2.size();
            ARAeps=ARAplanner->get_final_epsilon();

            delete ARAplanner;
            
            ///------------Store results---------//
            if DEBUG_P
            {
                env1.PrintTimeStat(stdout);
                trun.push_back(std::make_pair(MHAtime, ARAtime));  
                pathSize.push_back(std::make_pair(MHAlength,ARAlength));
                expansions.push_back(std::make_pair(MHAexpands,ARAexpands));
                eps.push_back(std::make_pair(MHAeps,ARAeps));
                cout<<"Expansions: "<<expansions.back().first<<" "<<expansions.back().second<<endl;
                cout<<"Time to Run: "<<trun.back().first<<" "<<trun.back().second<<endl;;
                cout<<"Path length: "<<pathSize.back().first<<" "<<pathSize.back().second<<endl;
                cout<<"Epsilon: "<<eps.back().first<<" "<<eps.back().second<<endl;
            }
        }
        
    }

    //Writing results from both searches to a file
    std::ofstream output_file("Results.txt");

    output_file<<"Expansions"<<endl;
    output_file<<"MHA     ARA"<<endl;

    for ( vector <pair<double,double> >::const_iterator it = expansions.begin() ; it != expansions.end(); it++){
        output_file<<it->first<<" "<<it->second<<endl;
    }
    output_file<<endl<<"Time"<<endl;
    output_file<<"MHA     ARA"<<endl;
    for ( vector <pair<double,double> >::const_iterator it = trun.begin() ; it != trun.end(); it++){
        output_file<<it->first<<" "<<it->second<<endl;
    }
    output_file<<endl<<"Path length"<<endl;
    output_file<<"MHA     ARA"<<endl;
    for ( vector <pair<double,double> >::const_iterator it = pathSize.begin() ; it != pathSize.end(); it++){
        output_file<<it->first<<" "<<it->second<<endl;
    }
    output_file<<endl<<"Epsilon"<<endl;
    output_file<<"MHA     ARA"<<endl;
    for ( vector <pair<double,double> >::const_iterator it = eps.begin() ; it != eps.end(); it++){
        output_file<<it->first<<" "<<it->second<<endl;
    }


}
 

 
int main(int argc, char *argv[])
{   
    double l1=0.01,w1=0.01,l2=0.02,w2=0.02;
    // set the perimeter of the robot   
    vector<sbpl_2Dpt_t> perimeter1, perimeter2;
    createFootprint(perimeter1,l1,w1); //Passed perimeter by reference
    // createFootprint(perimeter2,l2,w2); //Passed perimeter by reference

    // initialize an environment
    EnvironmentNAVXYTHETALAT env1;
    EnvironmentNAVXYTHETALAT env2;
    
    initializeEnv(env1, perimeter1,argv[1],argv[2]);
    initializeEnv(env2, perimeter1,argv[1],argv[2]);
    
    planxythetalat(argv[1],argv[2]);
}
