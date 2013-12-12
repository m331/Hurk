/* Test sketch for newping library and multiple sensors, both ultrasonic and infrared */

#include <NewPing.h>
#include <ros.h>
#include <ros/time.h>


/********************************************
                    SONAR
 ********************************************/
const float SONAR_CHARACTERISTICS[] = {0.01, 0.1, 2}; // FOV, min range, max range
#define SONAR_NUM     1 // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long lastPing = 0; // Holds time of last ping
byte actSon = 0;       // Holds number of active sonar

struct sonarData {
  unsigned long timestamp[SONAR_NUM]; // Array of timestamps
  unsigned int  cm[SONAR_NUM];        // Array with distance data (in cm)
} dat; // Make dat variable


NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(9, 10, MAX_DISTANCE) // Each sensor's trigger pin, echo pin, and max distance to ping.
};

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[actSon].check_timer())
  {
    dat.timestamp[actSon] = millis() - sonar[actSon].ping_result * 1000; // Set timestamp (beginning of SRF04 pulse)
    dat.cm[actSon] = sonar[actSon].ping_result / US_ROUNDTRIP_CM; // Set data
  }
}

void pingNextSonar()
{
    lastPing = millis();
    
    if ( ++actSon >= SONAR_NUM) // cur.actSon also gets incremented here!
    {
      actSon = 0;
      // Here, first check IR sensors and then:
      // Send data to ROS!
      
      /* Test print
      for (byte i=0; i < SONAR_NUM; i++)
      { Serial.print("#:"); Serial.print(i);
        Serial.print(" t:"); Serial.print(dat.timestamp[i]);
        Serial.print(" cm:"); Serial.println(dat.cm[i]);
      }/**/
    }
    
    sonar[actSon].timer_stop(); // Make sure previous timer is canceled before starting a new ping (insurance).
    dat.cm[actSon] = 0; // Make distance zero in case there's no ping echo for this sensor.
    sonar[actSon].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
}

/********************************************
                  INFRARED
 ********************************************/
#define IR_CHARACTERISTICS  [.01, .1, 2] // FOV, min range, max range


// ROS
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);

unsigned long range_timer;

void setup() {
  nh.initNode();
  nh.advertise(pub_range);
  
  
  //Serial.begin(9600);
  //Serial.println("HELLO");
  pinMode(8, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW); // pin 11 GND
  digitalWrite(8, HIGH); // pin 8 VCC
}

void send_range_ROS(unsigned long &timestamp, unsigned int &cm, byte type, const float sens_char_arr[]) {
  range_msg.radiation_type = type;
  
  char frameid[] = "/ir_ranger";
  range_msg.header.frame_id =  frameid;
  
  range_msg.field_of_view = sens_char_arr[0];//0.01;
  range_msg.min_range = sens_char_arr[1];//0.03;
  range_msg.max_range = sens_char_arr[2];//0.4;
  
  range_msg.range = cm;
  
  range_msg.header.stamp = nh.now();
  
  pub_range.publish(&range_msg);
  
  range_timer =  millis();
}

void loop() {
  if (millis() > lastPing + PING_INTERVAL)
  {
    byte curSon = actSon;
    pingNextSonar();
    send_range_ROS(dat.timestamp[curSon], dat.cm[curSon], sensor_msgs::Range::ULTRASOUND, SONAR_CHARACTERISTICS); // ULTRASOUND = 0 (msg header file define)
    //send_range_ROS(millis(), getir???, sensor_msgs::Range::INFRARED, IR_CHARACTERISTICS);
  }
  
  nh.spinOnce();
}



