#ifndef gplanner_h
#define gplanner_h

#include <iostream>
#include "ros/ros.h"
#include <vector>
#include <std_msgs/String.h>

//#include <geometry_msg/Pose.h>
//#include <geometry_msg/PoseStamped.h>
#include <tf/transform_broadcaster.h>
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

struct Pose
{
float x;
float y;

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
	geometry_msgs::PoseStamped posestamped;
	Pose pose;
};


ROShandle::ROShandle(ros::NodeHandle& n)
	{
	nh=n;
	init_ros();
	}

bool ROShandle::estimation(gplanner::OptimalSpotGenerator::Request &req, gplanner::OptimalSpotGenerator::Response &res)

{	

	if (req.request)
	{	
		ROS_INFO("Request Received from rEngine");
		//lplanner_costs.request = true;
		/*
		if(lplanner_client.call(lplanner_costs))
		{
			ROS_INFO("Response Received from lplanner");

		}

		else
		{
			ROS_INFO("Failed to get response from lPlanner");
		}
		
		*/
	res.spots[0]=0;
	res.spots[1]=0;
	res.spots[2]=0;
	
//	pose={0,0};
//	posestamped=PosetoPoseStamped(posestamped,pose);
//	res.finalSpots= posestamped;

	ROS_INFO("Response sent to rEngine");
	return true;
	}
}


void ROShandle::init_ros()
{
		optimalSpot = nh.advertiseService("OptimalSpotGenerator",&ROShandle::estimation,this); //initialise the ROS service for the spot query
		lplanner_client = nh.serviceClient<gplanner::SpotsTreadCost>("Local_Planner_Cost_Service");
	
	
}

// gPlanner::gPlanner(int nofspots,
// 		int qsize)
// {

// }




/*
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
*/
#endif	
