#include <iostream>
#include <gplanner.h>
#include <string>



int main( int argc, char ** argv)

{
	gPlanner planner; //create an object of the global planner
	
	ros::init (argc,argv,"Global_planner"); //initialise the ROS node	
	ros::NodeHandle nh;
	ROShandle ros(nh);
	
	ros::spin();


	return 0;

}