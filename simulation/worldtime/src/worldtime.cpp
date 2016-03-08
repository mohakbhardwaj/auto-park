#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <worldtime/timemsg.h>


using namespace std;

class time{

public:
	time()
	{
		t.hr=16;
		t.min=0;
	}
	
	void updateTime();
	
	worldtime::timemsg returntime()
		{	return t;}
	
	~time()
	{}


private:
	worldtime::timemsg t;

}wt;

int main(int argc, char** argv)

{

ros::init(argc,argv,"worldtime"); //initialise with node name worldtime

ros::NodeHandle nh;
ros::Publisher tpub=nh.advertise<worldtime::timemsg>("time",1); //publish time on topic name 'time'

ros::Rate loop_rate(10);

int k=1; //scaling factor for time

while (ros::ok())
{
		ros::Duration(k).sleep(); 
		wt.updateTime();
		tpub.publish(wt.returntime());
		//ROS_INFO("k secs elapsed");
		ros::spinOnce();
		loop_rate.sleep();

}




}

void time::updateTime()
{
	this->t.min+=1;
	if( this->t.min==60)
		{	this->t.min=0;
			this->t.hr+=1;
			if (this->t.hr==24)
				{
					this->t.hr=0;
				}
		}
	
}