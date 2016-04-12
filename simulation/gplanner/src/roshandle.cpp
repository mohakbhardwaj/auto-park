#include <roshandle.h>


#define CACHED 1

ROShandle::ROShandle(ros::NodeHandle& n)
{
	nh=n;
	init_ros();

	if (!CACHED)
	{	
		gp.startD2Exitplanner();
	}

	else
	{
		gp.useCache();
	}
}

bool ROShandle::estimation(gplanner::OptimalSpotGenerator::Request &req, gplanner::OptimalSpotGenerator::Response &res)

{	

	if (req.request)
	{	
		ROS_INFO("Request Received from rEngine");

		gp.getQuery(req.qval);
		
		// for(int i=0;i<5;i++);

		// if(lplanner_client.call(lplanner_costs))
		// {
		// 	ROS_INFO("Response Received from lplanner");

		// }

		// else
		// {
		// 	ROS_INFO("Failed to get response from lPlanner");
		// }
		

		int spotNo=gp.returnFinalSpot();
		cout<<endl<<"spot No "<<spotNo;
		struct envState e= gp.returnConfig(spotNo);
		cout<<endl<<"coords"<<e.x<<" "<<e.y<<" "<<e.th<<endl;
		res.spots[0]=e.x;
		res.spots[1]=e.y;
		res.spots[2]=e.th;
		ROS_INFO("Response sent to rEngine");
		return true;
	}
}


void ROShandle::init_ros()
{
		optimalSpot = nh.advertiseService("OptimalSpotGenerator",&ROShandle::estimation,this); //initialise the ROS service for the spot query
		lplanner_client = nh.serviceClient<gplanner::SpotsTreadCost>("Local_Planner_Cost_Service");


	}

