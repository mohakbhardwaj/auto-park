#ifndef gplanner_h
#define gplanner_h

#include <iostream>
#include "ros/ros.h"
#include <vector>
#include "gplanner/OptimalSpotGenerator.h" //include service files created
#include "gplanner/SpotsTreadCost.h"


struct spotstate
{
int x; //x coord of spot
int y;
bool occupied; 

int mseg_c; //cost of motion segment
int exit_c;


};

class gPlanner
{
public:
	gPlanner()
	{}
	// gPlanner(
	// 		//some input for an environment
	// 	int nofspots,
	// 	int qsize

	// 	); //will give an error if not initialised in cpp file


	// virtual void plan();
	

	// virtual void getTimeCosts();

	// virtual void RenderSpotService();
	
	// virtual void getplanrequest();
	
};

class ROShandle
{
public:

	ROShandle(ros::NodeHandle& n);
	
	bool estimation(gplanner::OptimalSpotGenerator::Request &req,gplanner::OptimalSpotGenerator::Response &res);
	void init_ros();


private:
	ros::NodeHandle nh;
	ros::ServiceServer optimalSpot;
	ros::ServiceClient lplanner_client;
	gplanner::SpotsTreadCost lplanner_costs;
};


ROShandle::ROShandle(ros::NodeHandle& n)
	{
	nh=n;
	
	}

bool ROShandle::estimation(gplanner::OptimalSpotGenerator::Request &req,gplanner::OptimalSpotGenerator::Response &res)

{	

	if (req.request)
	{	
		ROS_INFO("Request Received from rEngine");
		//lplanner_costs.request = true;

		if(lplanner_client.call(lplanner_costs))
		{
			ROS_INFO("Response Received from lplanner");

		}

		else
		{
			ROS_INFO("Failed to get response from lPlanner");
		}
		
	res.spots[0]=0;res.spots[1]=0;res.spots[2]=0;
	
	ROS_INFO("Response sent to rEngine");
	return true;
	}
}


void ROShandle::init_ros()
{
		optimalSpot = nh.advertiseService("OptimalSpotGenerator",&ROShandle::estimation,this); //initialise the ROS service for the spot query
		lplanner_client = nh.serviceClient<gplanner::SpotsTreadCost>("Local Planner Cost Service");
	
	
}

// gPlanner::gPlanner(int nofspots,
// 		int qsize)
// {

// }



#endif	