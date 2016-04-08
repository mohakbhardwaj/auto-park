#include <SoftwareSerial.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>

#define sRX 10
#define sTX 11
#define BUFSIZE 16
#define USB_CON

SoftwareSerial lrf= SoftwareSerial(sRX,sTX);
ros::NodeHandle  nh;

std_msgs::Int16 eStatetx, published;
ros::Publisher pub("EmergencyState", &eStatetx);

int get_state_from_IR();
float get_state_from_LRF();
void publish_state(std_msgs::Int16 eState);


float range;

char lrfData[BUFSIZE];  // Buffer for incoming data
char offset = 0;        // Offset into buffer

void setup()
{
  nh.initNode();
  nh.advertise(pub);
  //	delay(100);

  pinMode(sRX,INPUT);
  pinMode(sTX, OUTPUT);


  lrf.begin(9600);
  delay(50);
  lrf.print('U');
  while(lrf.read()!=':');
  delay(50);
  lrf.flush();
 
  published.data=-1;

  //Serial.begin(9600);


}



void loop()
{	

  range= get_state_from_LRF();
  //Serial.println(range);
  eStatetx.data= ((range<50 && range>15)||range==0) ?1:0;
  
  publish_state(eStatetx);


  nh.spinOnce();

}




float get_state_from_LRF()
{ 

  lrf.print('R');         

  // Arduino readBytesUntil() as an alternative 
  
  offset=0;
  
  lrfData[0] = 0;          

  float range;

  while(1)
  {

    if (lrf.available() > 0) 
    {
      lrfData[offset] = lrf.read();  
      if (lrfData[offset] == ':')          
      {
        lrfData[offset] = 0; 
        break;               
      }

      offset++;  
      if (offset >= BUFSIZE) offset = 0; 
    }

  }

  //Serial.println(lrfData);  
  //Serial.flush();             

  range= ((int)lrfData[5] -48)*100 + ((int)lrfData[6] -48)*10+ ((int)lrfData[7] -48) + ((int)lrfData[8] -48)* 0.1;
  //Serial.println(range);  
  return range;

}

void publish_state(std_msgs::Int16 eState)

{
  //Serial.println(eState); 
  if (published.data!=eState.data)
  { 
       published=eState;
       pub.publish(&eState);

  }


}

