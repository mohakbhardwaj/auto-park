
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;

//Code takes in IR Values from 3 sensors and applies Median filtering by taking 100 samples from each sensor, 
//Sorting them in an array and using the median values


sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);

const int analog_pins[] = {3,6,7};
unsigned long range_timer=0;
char frameid[] = "/ir_ranger";
int flag=0,distance[3],tdist=0,d;

float getRange(const int pin_num)

{
    int data = analogRead(pin_num)/4;
    // if the ADC reading is too low, 
    //   then we are really far away from anything
    if(data < 10)
        return 254;     // max range
    // Magic numbers to get cm
    int distance= 1309/(data-3);
    return distance; //convert to meters
}



void setup()
{
  nh.initNode();
  nh.advertise(pub_range);
  
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.03;
  range_msg.max_range = 0.4;
  
}

void loop()
{
  // publish the range value every 50 milliseconds
  //   since it takes that long for the sensor to stabilize
  flag=0;
  if ( (millis()-range_timer) > 100)
  
  {
    
    /*
    for( int j=0; j<3;j++)
    {
         d=0;
        d= getRange(analog_pins[j]);
        d+= getRange(analog_pins[j]);
        d+= getRange(analog_pins[j]);
        distance[j] =d/3;
        

         
    }
    */
    
    
    
    
   // if ((distance[0]>20) && (distance[1]>20) && (distance[2]>20))
   /*
     if(d>15)
        { tdist=35;}
        
        else
        
        {tdist =10;}
      
      */
      
//      for( int j=0;j<3;j++)
//      {
      tdist= getRange(analog_pins[1]);
      if(tdist<50)
      {
                range_msg.range=tdist;
                range_msg.header.stamp = nh.now();
                pub_range.publish(&range_msg);
      }
      //}
            
    }
    range_timer =  millis() + 100;
  
  nh.spinOnce();
}

