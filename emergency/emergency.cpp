#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>
#include <emergency/estate.h>

#include <sstream>


//std_msgs::String state;
ros::Publisher statePub;
emergency::estate msg;
int count,count2;


void rangeCheck( const sensor_msgs::Range::ConstPtr range_msg) 

	{ // Do something with range 


	// state.data= "7";

		if (float(range_msg->range) < 20 )

		{	

			// if( count==30)
			// {

		 	count=0;
				if (msg.state!=1)
				{
					msg.state=1;
					statePub.publish(msg);
					
					
				}
			// }
			
			// else
			// 		{count++;}

			
				//state.data= "1";
		}

		else
		{

			// if (count2==10)
			// {

			// 		count2=0;
				if (msg.state !=0)
				{
					msg.state=0;
					statePub.publish(msg);
					

				}
		// 	}
		// 		else
		// 			{count2++;}
				
			
		// 		// state.data="0";
		}



	}



	int main (int argc, char **argv)
	{

	ros::init(argc, argv, "emergency");  //Name of the node is emergency


	ros::NodeHandle nh;

	ros::Subscriber irSub = nh.subscribe("range_data", 1, rangeCheck);

	


	// statePub = nh.advertise<std_msgs::String>("emergencyState",1);  //tells the master that message type being published is a String on the topic emergencyState
	statePub = nh.advertise<emergency::estate>("emergencyState",1);  //tells the master that message type being published is a String on the topic emergencyState


	
	
	//statePub is the publisher instance, buffer of maximum 10 messages
	
	ros::Rate loop_rate(10); //need to sync this loop frequency with overall execution frequency
							// Currently at 10Hz
	
	

	msg.device="IR";
	msg.id=2;
	count=count2=0;
	msg.state=0;
	


	

	while (ros::ok())

	{

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
	

	//ros::spin();

}
