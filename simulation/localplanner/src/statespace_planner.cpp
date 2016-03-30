//System
#include <fstream>
#include <deque>
#include <queue>
#include <sstream>
#include <cstdlib>
//ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
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
const double PI  =3.141592653589793238463;
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
};
//Functor for comparing Nodes to be stored in priority queue
struct node_cmp_cost
{
   bool operator()( const Node *a, const Node *b ) const 
   {
    return a->f > b->f;
   }
};
//Functor for comparing two poses
struct pose_cmp
{
	bool operator()(const Pose a, const Pose b ) const
	{
		return abs(a.x + a.y + a.th) < abs(b.x + b.y + b.th); //Euclidean distance later

	}
};



//Struct for storing environment information
struct Environment
{
	double raster_size;
	std::string costMapFile, parkingSpotFile;
	std::vector<double> world_extents;
	std::vector<std::vector<int> > costmap; 
	std::map<Pose, std::string, pose_cmp> parking_spot_centers;

}env;

//std::priority_queue<Node*, std::vector<Node*>, node_cmp> Q;---> Note to self

//Function declarations
void openFile(std::ifstream&, std::string);
void LoadEnvironment(std::string, std::string );
void LoadCostMap(std::vector<std::vector<int> >&, std::string);
void LoadParkingSpots(std::map<Pose, std::string, pose_cmp>&, std::string);

bool processpathQuery(localplanner::optimPath::Request&, localplanner::optimPath::Response&);
bool processCostQuery(localplanner::spotsTreadCost::Request& , localplanner::spotsTreadCost::Response&);
double astarstatespace(std::pair<Pose,Pose>, std::vector<geometry_msgs::PoseStamped>&);
void end_pose(const Pose& startpose, Pose& endpose, double curvature, double length);
void lattice_pose(const Pose& continuouspose, Pose& latticepose, double raster);
bool poses_close(const Pose& p1, const Pose& p2);
double distance_euclidean(const Pose& p1, const Pose& p2);
double rad(double ); //converts angle from degree to radians
void get_movements(std::vector<std::pair<double,double> >& );
void PoseStampedtoPose(const geometry_msgs::PoseStamped&, Pose&);
void PosetoPoseStamped(geometry_msgs::PoseStamped&, const Pose&);
double h1(const Pose& p1, const Pose& p2);
//double h2();
//double h3();

int main(int argc, char **argv)
{
	LoadEnvironment("/home/pranav/catkin_ws/src/localplanner/env_files/env.txt", "../env_files/parking_spot.txt");
	//ROS node functionality
	ros::init(argc, argv, "statespace_planner");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	//ROS services setup
	ros::ServiceServer costservice = n.advertiseService("spotsTreadCost", processCostQuery);
	ros::ServiceServer pathservice = n.advertiseService("optimPath", processpathQuery);

		
		
	std::cout << "Running...Running...Running" << std::endl;
	
	loop_rate.sleep();

	ros::spin();
	

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
		} 

	}
}

void LoadEnvironment(std::string costMapFile, std::string parkingSpotFile)
{
	env.raster_size = 0.025;
	env.world_extents.push_back(40);
	env.world_extents.push_back(46);
	env.costMapFile = costMapFile;
	env.parkingSpotFile = parkingSpotFile;
	LoadCostMap(env.costmap, env.costMapFile);
	LoadParkingSpots(env.parking_spot_centers, env.parkingSpotFile);
	
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
		double nth = remainder(th + angle + PI,2*PI) - PI;
		endpose.x = nx;
		endpose.y = ny;
		endpose.th = nth;

	}
}
void get_movements(std::vector<std::pair<double,double> >& movements)
{	//Each movement is of the type (curvature,distance)
	double distance = 5.0;
	double curvature_min = 0.0;
	double curvature_max = 1.0/10;
	movements.push_back(std::make_pair(curvature_max, distance));
	movements.push_back(std::make_pair(curvature_min, distance));
	movements.push_back(std::make_pair(-curvature_max, distance));
	movements.push_back(std::make_pair(curvature_max, -distance));
	movements.push_back(std::make_pair(curvature_min, -distance));
	movements.push_back(std::make_pair(-curvature_max, -distance));
}
void lattice_pose(const Pose& continuouspose, Pose& latticepose, double pos_raster)
{	//Given a pose returns a discrete pose in lattice
	double heading_raster = rad(10.0);
	latticepose.x = (floor(continuouspose.x/pos_raster));
	latticepose.y = (floor(continuouspose.y/pos_raster));
	latticepose.th = (floor(continuouspose.th/heading_raster));

}


double distance_euclidean(const Pose& p1, const Pose& p2)
{
	//Returns euclidean distance between two poses
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

bool poses_close(const Pose& p1, const Pose& p2)
{
	//Returns true if two poses are close to each other
	//Tolerances have been hardcoded as 15 degrees for angle and 2.0 for position

	double d_angle = abs(remainder(p1.th - p2.th + PI, (2.0*PI)) - PI);

	return d_angle < rad(10.0) && distance_euclidean(p1,p2) <= 2.0;

}

double rad(double angle_in_degrees)
{
	return angle_in_degrees*(PI/180);
}
double h1(const Pose& p1, const Pose& p2)
{
	return distance_euclidean(p1,p2);
}
bool processpathQuery(localplanner::optimPath::Request &req, localplanner::optimPath::Response &res)
{	
	//TODO: Implement buffering of queries, multi-threading
	Pose start,goal;
	PoseStampedtoPose(req.query[0], start);
	PoseStampedtoPose(req.query[1], goal);
	ROS_INFO("Received path query from [%f, %f] to [%f, %f]", start.x, start.y, goal.x, goal.y);
	astarstatespace(std::make_pair(start,goal), res.path);
	return true;
}
bool processCostQuery(localplanner::spotsTreadCost::Request &req, localplanner::spotsTreadCost::Response &res)
{	
	//TODO: Implement buffering of queries, multi-threading
	Pose start,goal;
	PoseStampedtoPose(req.query[0], start);
	PoseStampedtoPose(req.query[1], start);
	ROS_INFO("Received cost query from [%f, %f] to [%f, %f]", start.x, start.y, goal.x, goal.y);
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
double astarstatespace(std::pair<Pose,Pose> query, std::vector<geometry_msgs::PoseStamped>& path)
{
	Pose start_pose = query.first;
	Pose goal_pose = query.second;
	Node *start = new Node;
	start->p = start_pose;
	start->g = 0.0001; start->f = start->g + h1(start->p, goal_pose);
	start->parent = NULL;
	
	double pose_raster = env.raster_size;
	double path_cost = 0; //Default value of cost to be returned at the end;
	std::priority_queue<Node*, std::vector<Node*>, node_cmp_cost> open;
	open.push(start);
	std::vector<double> extents = env.world_extents;
	std::set<Pose, pose_cmp> generated_states;
	std::vector<std::pair<double,double> >movements;
	get_movements(movements);
	Node *curr_node;
	while(open.size() > 0)
	{	//Stop search if frontier gets too large
		
		if(open.size() > 500000)
		{
			ROS_WARN("Timeout: Did not find feasible path for start and goal");
			break;
		}
		//Pop the best node from the frontier
		curr_node = open.top();
		open.pop();

		Pose curr_lattice_pose;
		lattice_pose(curr_node->p, curr_lattice_pose, pose_raster);
		if(generated_states.count(curr_lattice_pose))
		{
			continue;
		}
		//Insert popped pose into lattice
		generated_states.insert(curr_lattice_pose);
		//std::cout << "[x = " << curr_node.p.x << " y = " << curr_node.p.y << " th = " <<curr_node.p.th << std::endl;
		
		
		//Check if reached goal
		if(poses_close(curr_node->p, goal_pose))
		{
			path_cost = curr_node->g;	
			break; //Finished
		}

		//Expand the current node
		for(int i =0; i < movements.size(); ++i)
		{
			Node *new_node = new Node;
			double curvature = movements[i].first;
			double length = movements[i].second;
			end_pose(curr_node->p, new_node->p ,curvature, length);
			if(new_node->p.x < 0 || new_node->p.x >= extents[0] ||
			   new_node->p.y < 0 || new_node->p.y >= extents[1])
			{
				continue;
			}
			Pose new_lattice_pose;
			lattice_pose(new_node->p, new_lattice_pose, pose_raster);
			if(env.costmap[new_lattice_pose.x][new_lattice_pose.y] != 255)
			{
				new_node->g = curr_node->g + abs(length);
				double new_h = h1(new_node->p,goal_pose); //+ ...
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
			//std::cout << "[x = " << curr_node->p.x << " y = " << curr_node->p.y << " th = " <<curr_node->p.th << " ]" << std::endl;
			curr_node = curr_node->parent;
			
		}
		geometry_msgs::PoseStamped start_push, goal_push;
		PosetoPoseStamped(start_push, start_pose);
		PosetoPoseStamped(goal_push, goal_pose);
		path.insert(path.begin(),start_push);	
		path.push_back(goal_push);
	}
	else
	{
		ROS_WARN("Returning empty path");
		path.clear();
	}
	/*for(size_t i = 0; i < path.size(); ++i)
	{
		std::cout << "[x = " << path[i].x << " y = " << path[i].y << " th = " <<path[i].th << " ]" << std::endl;
	}*/
	return path_cost;
}