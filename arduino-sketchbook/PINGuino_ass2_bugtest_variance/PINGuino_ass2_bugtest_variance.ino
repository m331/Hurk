/* Test sketch for newping library and multiple sensors, both ultrasonic and infrared */

#include "RunningAverage.h"

#include <NewPing.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);


/********************************************
           SONAR DATA STRUCTURES
 ********************************************/
#define SONAR_NUM     3 // Number of sensors.
#define MAX_DISTANCE 200.0 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define RA_SIZE 3 // Running average size

const float SONAR_CHARACTERISTICS[] = {0.5, 5.0, MAX_DISTANCE}; // FOV, min range, max range

unsigned long lastPing = 0; // Holds time of last ipng
byte actSon = 0;       // Holds number of active sonar

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(5,5, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(6,6, MAX_DISTANCE),
  NewPing(7,7, MAX_DISTANCE)
};

RunningAverage myRA[SONAR_NUM] = {
  RunningAverage(RA_SIZE), // Running average
  RunningAverage(RA_SIZE),
  RunningAverage(RA_SIZE)
};


void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[actSon].check_timer())
  {
    myRA[actSon].addValue( float(sonar[actSon].ping_result) / US_ROUNDTRIP_CM ); // Add to running average buffer
  }
}
void pingNextSonar()
{
    lastPing = millis();
    
    if ( ++actSon >= SONAR_NUM) // actSon also gets incremented here!
    {
      actSon = 0;
      // Here, first check IR sensors and then:
      // Send data to ROS!
      
      /* Test print
      for (byte i=0; i < SONAR_NUM; i++)
      { Serial.print("#:"); Serial.print(i);
        Serial.print(" t:"); Serial.print(dat.timestamp[i]);
        Serial.print(" cm:"); Serial.println(dat.cm[i]);
      }*/
      /* dim LED if valid data
      if (moving_avg_cm[0] > 8.0)
      {//Serial.println("PENIS");
        pinMode(13,OUTPUT);
        digitalWrite(13,LOW);}
      else digitalWrite(13,HIGH);
      /**/
    }
    
    sonar[actSon].timer_stop(); // Make sure previous timer is canceled before starting a new ping (insurance).
    //moving_avg_cm[0] = 0; // Make distance zero in case there's no ping echo for this sensor.
    sonar[actSon].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
}

void send_sonar_range_ROS(float cm, const float sens_char_arr[], int n) {
  range_msg.radiation_type = n;//sensor_msgs::Range::ULTRASOUND; // ULTRASOUND = 0 (msg header file define)
  
  char frameid[] = "/ultrasonic_ranger";
  range_msg.header.frame_id =  frameid;
  
  range_msg.field_of_view = sens_char_arr[0];//0.01;
  range_msg.min_range = sens_char_arr[1];//0.03;
  range_msg.max_range = sens_char_arr[2]; //MAX_DISTANCE + float(n);
  
  range_msg.range = cm;
  
  range_msg.header.stamp = nh.now(); // TODO timestamp???
  
  pub_range.publish(&range_msg);
}/**/

void setup() {
  nh.initNode();
  nh.advertise(pub_range);
  
  myRA[actSon].clear(); // explicitly start clean
  //Serial.begin(9600);
  //Serial.println("HELLO");
  //pinMode(8, OUTPUT);
  //pinMode(11, OUTPUT);
  //digitalWrite(11, LOW); // pin 11 GND
  //digitalWrite(8, HIGH); // pin 8 VCC
}

void loop() {
  if (millis() > lastPing + PING_INTERVAL)
  {
    //float avg_cm;
    //for (byte i=0; i < MOVING_AVG; i++)
      //avg_cm += moving_avg_cm[i];
    //avg_cm /= MOVING_AVG;
    // Variance calculation
    //float var = calcVar();
    //for (byte i=0; i < MOVING_AVG; i++)
    send_sonar_range_ROS( myRA[actSon].getAverage() , SONAR_CHARACTERISTICS , actSon );
    pingNextSonar();
  }
  
  nh.spinOnce();
}



