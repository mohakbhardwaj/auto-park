//System
#include <fstream>
#include <deque>
#include <queue>
#include <sstream>
#include <cstdlib>
#include <cmath>
//ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <ros/console.h>
//Local
#include <localplanner/spotsTreadCost.h>
#include <localplanner/optimPath.h>
//Not required
/*#include <iostream>
#include <vector>
#include <utility>
#include <string>
#include <set>
#include <map>
#include <cmath>

*/

//TODO: Make class for Pose with conversions for posestamped, Node, Environment with motion model and primitives
//TODO: Make different class for lattice_tree with Node types, generated states etc.
//TODO: Add costMap2D from ROS 
const double PI  = 3.141;
//PI  =3.141592653589793238463

//Struct for Nodes in A* tree traversal
struct Pose
{
	double x, y, th;
};
struct Node
{	Pose p; //Pose information
	double g, f; //Cost information
	Node *parent; //Used for generating path in the end
	//Pose previous;
	int nid;
	Pose latticepose;
};
//Functor for comparing Nodes to be stored in priority queue
struct node_cmp_cost
{
   bool operator()( const Node *a, const Node *b ) const 
   {
    return a->f > b->f;
   }
};
double rad(double);
double modulo (double , double );
//Functor for comparing two poses
struct pose_cmp
{
	bool operator()(const Pose a, const Pose b ) const
	{
		// double d = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2)); 
		// double d_angle = abs(modulo(p1.th - p2.th + PI, (2.0*PI)) - PI);
		// return d_angle < rad(1.0) && d <= 0.1;
		 // return (a.x + a.y + a.th) < (b.x + b.y + b.th); //Euclidean distance later
		return a.x < b.x && a.y < b.y && a.th < b.th;
	}
};



//Struct for storing environment information
struct Environment
{
	double raster_size;
	std::string costMapFile, parkingSpotFile;
	std::vector<std::pair<double, double> > world_extents;
	std::vector<std::vector<int> > costmap; 
	int dimension;
	std::vector<int> num_cells;
	std::vector<int> num_cells_multiply;
	std::map<Pose, std::string, pose_cmp> parking_spot_centers;

}env;

//std::priority_queue<Node*, std::vector<Node*>, node_cmp> Q;---> Note to self

//Function declarations
void openFile(std::ifstream&, std::string);
void LoadEnvironment(std::string, std::string );
void LoadCostMap(std::vector<std::vector<int> >&, std::string);
void LoadParkingSpots(std::map<Pose, std::string, pose_cmp>&, std::string);
int checkCollision(const Pose&); //Return collision cost value
int extendCheckCollision(const Pose& , double , double, double );
bool processpathQuery(localplanner::optimPath::Request&, localplanner::optimPath::Response&);
bool processCostQuery(localplanner::spotsTreadCost::Request& , localplanner::spotsTreadCost::Response&);
double astarstatespace(std::pair<Pose,Pose>, std::vector<geometry_msgs::PoseStamped>&);
void end_pose(const Pose& startpose, Pose& endpose, double curvature, double length);
void lattice_pose(const Pose& continuouspose, Pose& latticepose, double raster);
bool poses_close(const Pose& p1, const Pose& p2);
double distance_euclidean(const Pose& p1, const Pose& p2);
 //converts angle from degree to radians
void get_movements(std::vector<std::pair<double,double> >& );
void PoseStampedtoPose(const geometry_msgs::PoseStamped&, Pose&);
void PosetoPoseStamped(geometry_msgs::PoseStamped&, const Pose&);
double h1(const Pose& , const Pose& ); //Go towards goal
double h2(const Pose&); //Stay away from obstacles
int gridCoordinateToNodeId(const Pose&);

//double h2();
//double h3();

int main(int argc, char **argv)
{
	LoadEnvironment("/home/shivam/catkin_ws/src/localplanner/env_files/env.txt", "/home/shivam/catkin_ws/src/localplanner/env_files/parking_spot.txt");
	//ROS node functionality
	ros::init(argc, argv, "statespace_planner");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	//debug print of costmap
	/*for(size_t i = 0; i < env.costmap.size(); ++i)
	{
		for(size_t j =0; j < env.costmap[i].size(); ++j)
		{	std::cout << env.costmap[i][j];

		}
		std::cout << std::endl;
	}*/
	//std::cout << env.costmap.size() << " " << env.costmap[0].size() << std::endl;
	/*for(size_t j = 0; j < env.costmap[0].size(); ++j)
	{	
		std::cout << env.costmap[0][j] << " ";

	}*/
	// Pose s;
	// Pose g;
	// s.x = 2.5;
	// s.y =  2;
	// s.th = PI/2;

	// g.x = 26.25; g.y = 22.25; g.th = (-1.0*PI)/2;
	// std::vector<geometry_msgs::PoseStamped> path;
	// double dd =  astarstatespace(std::make_pair(s,g),path);
	// // Pose l;
	// // lattice_pose(s, l, env.raster_size);
	// // std::cout << l.x << " " << l.y << " " << " " << checkCollision(s) << std::endl;


	//ROS services setup
	ros::ServiceServer costservice = n.advertiseService("spotsTreadCost", processCostQuery);
	ros::ServiceServer pathservice = n.advertiseService("optimPath", processpathQuery);
	
	loop_rate.sleep();

	ros::spin();
	

}

double modulo (double m, double n) //Coz C++ you hearltess bitch!!! 
{ 
	// return m >= 0 ? m % n : ( n - abs ( m%n ) ) % n; 
	return m >= 0 ? fmod(m,n) : fmod(( n - abs ( fmod(m,n) ) ) , n); 
}
//TODO: Make generic by modifying pose
int gridCoordinateToNodeId(Pose& p)
{
	   int node_id = 0;
	   double x = p.x;
	   double y = p.y;
	   double th = p.th;

	   // int numCellsX = 
    //     for(i = 0; i < env.dimension; ++i)
    //     {
    //     	node_id += 

    //     }
        node_id += x*env.num_cells_multiply[0];
        node_id += y*env.num_cells_multiply[1];
        node_id += th*env.num_cells_multiply[2];
        //     mul = 1
        //     for j in range(self.dimension - i-1):
        //         mul = mul*self.num_cells[j]
        //     node_id = node_id + coord[self.dimension - i-1]*mul
        //     node_id = int(node_id)

        return node_id;
}
void openFile(std::ifstream& file, std::string filename)
{
	while(true)
	{
		file.open(filename.c_str());
		if(file.is_open())
		{
			break;
		}
		else
		{
			std::cerr<<"No such file exists with name "<< filename << std::endl;
			file.clear();
			ros::Duration(0.1).sleep(); //sleep coz blaaahhhhh
		} 

	}
}

void LoadEnvironment(std::string costMapFile, std::string parkingSpotFile)
{
	////////////////RASTER SIZE THAT DEFINITELY WORKS///////////
	env.raster_size = 0.1;
	//////////////////////////////////////////////////////////
	env.world_extents.push_back(std::make_pair(0, 42.5));
	env.world_extents.push_back(std::make_pair(0, 48));
	env.world_extents.push_back(std::make_pair(-PI, PI));
	env.costMapFile = costMapFile;
	env.parkingSpotFile = parkingSpotFile;
	LoadCostMap(env.costmap, env.costMapFile);
	LoadParkingSpots(env.parking_spot_centers, env.parkingSpotFile);
	env.dimension = 3;
	for(int i =0; i < env.dimension; ++i)
	{
		double currLower = env.world_extents[i].first;
		double currUpper = env.world_extents[i].second;
		int currNumCells =  (int)((currUpper - currLower)/env.raster_size);
		env.num_cells.push_back(currNumCells);
	} 
	for(int i = 0; i < env.dimension; ++i)
	{
		int w = 1;
		for(int j = 0; j < i; ++j)
		{
			w *= env.num_cells[i];
		}
		env.num_cells_multiply.push_back(w);
	}
	
}
void LoadCostMap(std::vector<std::vector<int> >& costmap, std::string costMapFileName)
{
	std::ifstream costMapFile;
	openFile(costMapFile, costMapFileName);
	std::string costString;

	/*for( int i = 0; i < 1920; ++i)
	{
		std::vector<int> temp;
		for(int j = 0; j < 1700; ++j)
		{
			temp.push_back(0);
		}
		costmap.push_back(temp);
	}*/

	while(getline(costMapFile,costString))
	{	
		std::stringstream s(costString);
		int cellVal;     //int for now....maybe double later?
		std::vector<int> temp;
		while(s >> cellVal)
		{
			temp.push_back(cellVal);
		}
		costmap.push_back(temp);
	}
}
void LoadParkingSpots(std::map<Pose, std::string, pose_cmp>& parking_spot_centers, std::string parkingSpotFileName)
{
	std::ifstream parkingSpotFile;
	//openFile(parkingSpotFile, parkingSpotFileName);
	Pose dummy;
	dummy.x = 0;
	dummy.y = 0;
	dummy.th = 0;
	parking_spot_centers[dummy] = "Full"; 
}
void end_pose(const Pose& startpose, Pose& endpose, double curvature, double length)
{
	//Returns end pose using bicycle model given start pose, curvature and length travelled
	double x = startpose.x;
	double y = startpose.y;
	double th = startpose.th;

	if(curvature == 0)
	{	//Linear Motion
		x += length*cos(th);
		y += length*sin(th);
		endpose.x = x;
		endpose.y = y;
		endpose.th = th;
	}
	else
	{
		//Motion along curve segment
		double tx = cos(th);
		double ty = sin(th);
		double radius = 1.0/curvature;
		double xc = x - radius*ty; //Center of circle
		double yc = y + radius*tx;
		double angle = length/radius;
		double cosa = cos(angle);
		double sina = sin(angle);
		double nx = xc + radius*(cosa*ty + sina*tx);
		double ny = yc + radius*(sina*ty - cosa*tx);
		//double nth;
		// if((th + angle )> PI)
		// {
		// 	nth = th + angle - 2*PI;
		// }
		// else if((th + angle )< -1.0*PI)
		// {
		// 	nth = th + angle + 2*PI;
		// }
		// else
		// {
		// 	nth = th + angle;
		// }
	    double nth = modulo(th + angle + PI,2*PI)- PI;
		//double nth = fmod(th + angle + PI,2*PI) + PI;
		//double nth = (th + angle + PI)%(2*PI) - PI;

		endpose.x = nx;
		endpose.y = ny;
		endpose.th = nth;
		// std::cout << startpose.th << " " << curvature << " " << endpose.th << " " << angle << std::endl;
		// std::cout << endpose.th << std::endl;
	}

}
//Check in the raster or costmap for the value of collision magnitude
int checkCollision(const Pose& p)
{	
	Pose lp; 
	lattice_pose(p, lp, env.raster_size);
	return env.costmap[env.costmap.size()-lp.y-1][lp.x];
}
int extendCheckCollision(const Pose& startpose, double curvature, double length, double dl)
{
	double l = 0.0;
	dl = copysign(dl, length);
	// std::cout << dl << " " << length << std::endl;
	int coll_val = 0;
	//std::cout << "l = " << l << " length = " << length << " dl = " << dl <<  std::endl;
	// if(dl < 0)
	// {
	// 	for(size_t i = 0; i < env.costmap.size(); ++i)
	// 	{
	// 		for(size_t j =0; j < env.costmap[i].size(); ++j)
	// 		{	std::cout << env.costmap[i][j];

	// 		}
	// 		std::cout << std::endl;
	// 	}	
	// }
	while(abs(l) < length)
	{
		Pose e;
		end_pose(startpose, e, curvature, l);
	    coll_val = checkCollision(e);
		if(coll_val == 255)
		{

			break;
		}
		l += dl;

	}
	//std::cout << coll_val << std::endl;
	return coll_val;

}
void get_movements(std::vector<std::pair<double,double> >& movements)
{	//Each movement is of the type (curvature,distance)
	
	///////////////////////////MOTION PRIMITIVES THAT DEFINITELY WORK/////////////////////////
	double distance = 1;
	double curvature_min = 0.0;
	double curvature_max = 1.0/4.0;
	movements.push_back(std::make_pair(curvature_max, distance));
	movements.push_back(std::make_pair(curvature_min, distance));
	movements.push_back(std::make_pair(-1.0*curvature_max, distance));
	movements.push_back(std::make_pair(curvature_max, -1.0*distance));
	movements.push_back(std::make_pair(curvature_min, -1.0*distance));
	movements.push_back(std::make_pair(-1.0*curvature_max, -1.0*distance));
	movements.push_back(std::make_pair(2*curvature_max, distance));
	movements.push_back(std::make_pair(-2*curvature_max, distance));
	movements.push_back(std::make_pair(3*curvature_max, distance));
	movements.push_back(std::make_pair(-3*curvature_max, distance));
	movements.push_back(std::make_pair(4.0*curvature_max, distance));
	movements.push_back(std::make_pair(-4.0*curvature_max, distance));
	movements.push_back(std::make_pair(5.0*curvature_max, 0.5*distance));
	movements.push_back(std::make_pair(-5.0*curvature_min, 0.5*distance));
	
	////////////////////////////////////////////////////////////////////////////

	// movements.push_back(std::make_pair(3.0*curvature_max, 0.5*distance));
	// // movements.push_back(std::make_pair(3.0*curvature_min, distance));
	// movements.push_back(std::make_pair(-3.0*curvature_max, 0.5*distance));
	// movements.push_back(std::make_pair(curvature_max, 2*distance));
	// movements.push_back(std::make_pair(curvature_min, 2*distance));
	// movements.push_back(std::make_pair(-1.0*curvature_max, 2*distance));
}
void lattice_pose(const Pose& continuouspose, Pose& latticepose, double pos_raster)
{	//Given a pose returns a discrete pose in lattice
	// double heading_raster = rad(0.025);

	latticepose.x = (floor((continuouspose.x - env.world_extents[0].first)/env.raster_size));
	latticepose.y = (floor((continuouspose.y - env.world_extents[1].first)/env.raster_size));
	latticepose.th = (floor((continuouspose.th - env.world_extents[2].first)/rad(0.1)));
}


double distance_euclidean(const Pose& p1, const Pose& p2)
{
	//Returns euclidean distance between two poses
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2)); //add th ????
}

bool poses_close(const Pose& p1, const Pose& p2)
{
	//Returns true if two poses are close to each other
	//Tolerances have been hardcoded as 15 degrees for angle and 2.0 for position

	double d_angle = abs(modulo(p1.th - p2.th + PI, (2.0*PI)) - PI);
	// d_angle < rad(15.0) &&
	////////////////////////DEFINITELY WORKS/////////////////////////
	return  d_angle < rad(45) && distance_euclidean(p1,p2) <= 2;
	//////////////////////////////////////////////////////



	// double d = distance_euclidean(p1,p2);
	// std::cout << d << std::endl;
	// return d <= 1.0;
	//return distance_euclidean(p1,p2) <= 2.0;
}

double rad(double angle_in_degrees)
{
	return angle_in_degrees*(PI/180);
}
double h1(const Pose& p1, const Pose& p2)
{
	return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)); // + pow(p2.th - p1.th, 2));
	// return abs(p2.x - p1.x) + abs(p2.y - p1.y);
}
double h2(const Pose& p)
{
	return env.costmap[env.costmap.size()-p.y-1][p.x];
}
bool processpathQuery(localplanner::optimPath::Request &req, localplanner::optimPath::Response &res)
{	
	//TODO: Implement buffering of queries, multi-threading
	Pose start,goal;
	PoseStampedtoPose(req.query[0], start);
	PoseStampedtoPose(req.query[1], goal);
	ROS_INFO("Received path query from rendering engine for [%f, %f] to [%f, %f]", start.x, start.y, goal.x, goal.y);
	astarstatespace(std::make_pair(start,goal), res.path);
	return true;
}
bool processCostQuery(localplanner::spotsTreadCost::Request &req, localplanner::spotsTreadCost::Response &res)
{	
	//TODO: Implement buffering of queries, multi-threading
	Pose start,goal;
	PoseStampedtoPose(req.query[0], start);
	PoseStampedtoPose(req.query[1], start);
	ROS_INFO("Received cost query from global planner for [%f, %f] to [%f, %f]", start.x, start.y, goal.x, goal.y);
	std::vector<geometry_msgs::PoseStamped> path;
	res.pathcost = astarstatespace(std::make_pair(start,goal), path);

	return true;
}

void PoseStampedtoPose(const geometry_msgs::PoseStamped& p1, Pose& p2)
{
	p2.x =  p1.pose.position.x;
	p2.y = p1.pose.position.y;
	tf::Quaternion p1_quat = tf::Quaternion(p1.pose.orientation.x,p1.pose.orientation.y,
											p1.pose.orientation.z, p1.pose.orientation.w);
	p2.th = tf::getYaw(p1_quat);

}
void PosetoPoseStamped(geometry_msgs::PoseStamped& p1, const Pose& p2)
{
	
	tf::Quaternion p1_quat = tf::createQuaternionFromYaw(p2.th);
			
	p1.pose.position.x = p2.x;
    p1.pose.position.y = p2.y;
      		
    p1.pose.orientation.x = p1_quat.x();
    p1.pose.orientation.y = p1_quat.y();
    p1.pose.orientation.z = p1_quat.z();
    p1.pose.orientation.w = p1_quat.w();
}
//TODO: Remove need for latticepose in node
// TODO: Make checking for world extents generic for dimensions
double astarstatespace(std::pair<Pose,Pose> query, std::vector<geometry_msgs::PoseStamped>& path)
{	ros::Time begin = ros::Time::now();
	double pose_raster = env.raster_size;
	Pose start_pose = query.first;
	Pose goal_pose = query.second;
	Node *start = new Node;
	start->p = start_pose;
	lattice_pose(start->p, start->latticepose, pose_raster);
	start->g = 0.0001; start->f = start->g + h1(start->p, goal_pose) + h2(start->latticepose);
	start->parent = NULL;
	
	start->nid = gridCoordinateToNodeId(start->latticepose);
	double path_cost = 0; //Default value of cost to be returned at the end;
	std::priority_queue<Node*, std::vector<Node*>, node_cmp_cost> open;
	open.push(start);
	// std::vector<std::pair<double, double> > extents = env.world_extents;
	std::set<int> visited;
	std::vector<std::pair<double,double> >movements;
	get_movements(movements);
	//DEBUG: Print open
	// for(std::priority_queue<>::iterator it = q.begin(); it != q.end(); ++it)
	// {

	// }
	// std::vector<std::vector<int> > visited;
	// for(size_t i = 0; i < env.costmap.size(); ++i)
	// {	std::vector<int> temp;
	// 	for(size_t j =0; j < env.costmap[i].size(); ++j)
	// 	{	
	// 		temp.push_back(0);
	// 	}
	// 	visited.push_back(temp);
	// }
	
	Node *curr_node;
	if(checkCollision(goal_pose) == 255)
	{
		ROS_WARN("Invalid query: goal in collison. Returning empty path");
		path.clear();
		return -1;

	}
	while(open.size() > 0)
	{	//Stop search if frontier gets too large
		// std::cout << "Still searching" << std::endl;
		if(open.size() > 500000)
		{
			
			break;
		}
		//Pop the best node from the frontier
		curr_node = open.top();
		open.pop();

		// Pose curr_lattice_pose;
		// lattice_pose(curr_node->p, curr_lattice_pose, pose_raster);
		// if(generated_states.count(curr_lattice_pose))
		if(visited.count(curr_node->nid))
		{
			
			continue;
		}
		
		//Insert popped pose into lattice
		// generated_states.insert(curr_lattice_pose);
		visited.insert(curr_node->nid);
		
		//Check if reached goal
		if(poses_close(curr_node->p, goal_pose))
		{
			std::cout << curr_node->p.x << " " << curr_node->p.y << std::endl;
			path_cost = curr_node->g;	
			
			break; //Finished
		}
		// std::cout << curr_node->p.x << " " << curr_node->p.y << std::endl;
		//Expand the current node
		for(int i = 0; i < movements.size(); ++i)
		{
			Node *new_node = new Node;
			double curvature = movements[i].first;
			double length = movements[i].second;

			end_pose(curr_node->p, new_node->p ,curvature, length);
			if(new_node->p.x < env.world_extents[0].first || new_node->p.x >= env.world_extents[0].second ||
			   new_node->p.y < env.world_extents[1].first || new_node->p.y >= env.world_extents[1].second)
			{
				continue;
			}
			
			lattice_pose(new_node->p, new_node->latticepose,pose_raster);
			new_node->nid = gridCoordinateToNodeId(new_node->latticepose);
			// std::cout << new_node->p.x << " " << new_node->p.y << " " << new_node->p.th << std::endl;
			// std::cout << new_lattice_pose.x << " " << new_lattice_pose.y << std::endl;
			// //std::cout << env.costmap[new_lattice_pose.y][new_lattice_pose.x] << std::endl;
			// if(generated_states.count(new_lattice_pose))
			// {
			// 	continue;
			// }
			if(visited.count(new_node->nid))
			{
				// std::cout << new_node->nid <<std::endl;
				// std::cout << new_node->p.x << " " << new_node->p.y << " " << new_node->p.th << std::endl;
				continue;
			}
		//Insert 

			if(extendCheckCollision(curr_node->p, curvature, length, pose_raster) != 255)
			{	//std::cout << extendCheckCollision(curr_node->p, curvature, length, pose_raster)<< std::endl;
			
				///////////////////////////WEIGHTS THAT DEFINITELY WORK////////////////////////
				new_node->g = curr_node->g + abs(length) + h2(new_node->latticepose)/64.0;
				double new_h = h1(new_node->p,goal_pose); //+ ...
				/////////////////////////////////////////////////////////////////////////////
				new_node->f = new_node->g + new_h;
				new_node->parent = curr_node;
				open.push(new_node);
			}

		}

	}

	if(poses_close(curr_node->p, goal_pose))
	{
		path.clear();
		std::cout <<"Forming path " << std::endl;

		while(curr_node->parent != NULL)
		{	
			geometry_msgs::PoseStamped pose_push;
			PosetoPoseStamped(pose_push, curr_node->p);
			path.insert(path.begin(),pose_push);
			// std::cout << "[x = " << curr_node->p.x << " y = " << curr_node->p.y << " th = " <<curr_node->p.th << " ]" << std::endl;
			curr_node = curr_node->parent;
			
		}
		geometry_msgs::PoseStamped start_push, goal_push;
		PosetoPoseStamped(start_push, start_pose);
		PosetoPoseStamped(goal_push, goal_pose);
		path.insert(path.begin(),start_push);	
		path.push_back(goal_push);
		ros::Duration elapsed_time = ros::Time::now() - begin;
		ROS_INFO_STREAM("Path returned in " << elapsed_time << " seconds");
	}
	else
	{
		// std::cout << "Closed Size: " << generated_states.size() << std::endl;
		if(open.size() == 0)
		{
			ROS_WARN("Exhausted all options. Returning empty path");
		}
		else if(open.size() > 500000)
		{
			ROS_WARN("Timeout: No path found. Returning empty path");
		}

		
		
		path.clear();
		return -1;
	}
	/*for(size_t i = 0; i < path.size(); ++i)
	{
		std::cout << "[x = " << path[i].x << " y = " << path[i].y << " th = " <<path[i].th << " ]" << std::endl;
	}*/
	return path_cost;
}
