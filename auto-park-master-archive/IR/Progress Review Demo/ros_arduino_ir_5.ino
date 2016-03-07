// Dorothy Kirlew
// Using rosserial_arduino_demos

 
#include<stdlib.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String ir_msg;
ros::Publisher ir_range("range_data", &ir_msg);
char irVAL[13]; // this HAS to be a big number here, regardless of the actual size of the value
char result[15];

const int analog_pins[] = {0, 2, 4};
float readings[3];
char irName[2];
char final[20];
char temp[10];

const int danger_zone = 15;

boolean tooClose;

unsigned long range_timer;

// converts IR output to cm
float getRange(int pin_num)
{
    int sample;
    sample = analogRead(pin_num)/4;
   
    // if reading is too low, then we're super far away
    if(sample < 10)
    {
      // max range
      // TODO: not sure about 254 being the max range... should be 10-80 cm
      // actually doesn't matter.  as long as it's > danger_zone, ultimately ignored
        return 254;
    }
    // Magic numbers to get cm
    sample= 1309/(sample-3);
    //return (sample - 1)/100; //convert to meters
    return sample;
}


void setup()
{
  // initialize node
  // publish data on ir_range
  nh.initNode();
  nh.advertise(ir_range);
}

void loop()
{
  // reset tooClose
  tooClose = false;
  result[0]='\0';
 
  // publish the range value every 50 milliseconds
  //   since it takes that long for the sensor to stabilize
  if ( (millis()-range_timer) > 50)
  {
    int i = 0;

    // loop through three IRs
    while(i < 3 && !tooClose)
    {
      readings[i]=getRange(analog_pins[i]);
      // if IR is reading value within danger_zone, publish data
      if(readings[i] <= danger_zone)
      {
        
        dtostrf(readings[i], 2, 3, temp);
        
        sprintf(final,"#%i-%s", i, temp);
        
        ir_msg.data = final;
       
        ir_range.publish(&ir_msg);
        // tooClose!!!
        tooClose = true;
      }
      i++;
    }
    range_timer =  millis();
  }
  nh.spinOnce();
}
