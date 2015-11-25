#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>


#include <sstream>


std_msgs::String state;


void rangeCheck( const sensor_msgs::Range::ConstPtr range_msg) 

{ // Do something with range 

	ROS_INFO("I heard something");
	// state.data= "7";

		if (float(range_msg->range) < 20 )
	
			{
				state.data= "1";
			}
	
		else
			{
				state.data="0";
			}

			

	}



int main (int argc, char **argv)
{

	ros::init(argc, argv, "emergency");  //Name of the node is emergency


	ros::NodeHandle nh;

	ros::Subscriber irSub = nh.subscribe("range_data", 1, rangeCheck);

	


	ros::Publisher statePub = nh.advertise<std_msgs::String>("emergencyState",1);  //tells the master that message type being published is a String on the topic emergencyState


	
	
	//statePub is the publisher instance, buffer of maximum 10 messages
	
	ros::Rate loop_rate(10); //need to sync this loop frequency with overall execution frequency
							// Currently at 10Hz
	ROS_INFO("I heard 2 somethings ");
	

	
	


	

	while (ros::ok())

	{

		statePub.publish(state);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;

	//ros::spin();

}
