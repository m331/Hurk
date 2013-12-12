/* Test sketch for newping library and multiple sensors, both ultrasonic and infrared */

#include <NewPing.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);

/********************************************
                  INFRARED
 ********************************************/
#define IR_CHARACTERISTICS  [.01, .1, 2] // FOV, min range, max range

// Sensor pin declarations
// TODO

// Read sensor at pin n
unsigned int readIRsensor(byte pin)
{
  byte l = 5; // Number of reads. Must be odd!
  unsigned int val[l];
  for (byte i=0; i<l; i++)
  {
    val[i] = analogRead(pin);
  }
  //Serial.print("Val list: [");
  //for (byte i=0; i<l; i++) {Serial.print(val[i]); Serial.print(" ");}
  //Serial.println("]");
  
  // Sort list, using simple bubble sort
  byte w = 1;
  while (w != 0)
  {
    w = 1; // At start of array, set w=1
    for (byte i = 0; i<(l-1); i++) // Loop through array
    {
      if (val[i+1] < val[i]) // SWAP
      {
        w = 2; // If swap is made, set w=2
        unsigned int temp = val[i];
        val[i] = val[i+1];
        val[i+1] = temp;
      }
    }
    if (w==1) w = 0; // If at end of list w is still 1, no swaps were made, so we end the loop
  }
  //Serial.print("Val: "); Serial.println(val[l/2]);
  // CONVERSION
  return val[l/2]; // Return median (middle value)
}


/********************************************
           SONAR DATA STRUCTURES
 ********************************************/
#define SONAR_NUM     1 // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

const float SONAR_CHARACTERISTICS[] = {0.01, 0.1, 2}; // FOV, min range, max range

unsigned long lastPing = 0; // Holds time of last ipng
byte actSon = 0;       // Holds number of active sonar

#define MOVING_AVG 5
float moving_avg_cm[MOVING_AVG] = {0,0,0,0,0};

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(9, 10, MAX_DISTANCE) // Each sensor's trigger pin, echo pin, and max distance to ping.
};


void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[actSon].check_timer())
  {
    for (int i=(MOVING_AVG-1); i >= 1; i--) // Move buffer
      moving_avg_cm[i] = moving_avg_cm[i - 1];
    moving_avg_cm[0] = float(sonar[actSon].ping_result) / US_ROUNDTRIP_CM; // Set data
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
      }*/
      // dim LED if valid data
      if (moving_avg_cm[0] > 8.0)
      {//Serial.println("PENIS");
        pinMode(13,OUTPUT);
        digitalWrite(13,LOW);}
      else digitalWrite(13,HIGH);
      /**/
    }
    
    sonar[actSon].timer_stop(); // Make sure previous timer is canceled before starting a new ping (insurance).
    moving_avg_cm[0] = 0; // Make distance zero in case there's no ping echo for this sensor.
    sonar[actSon].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
}

void send_sonar_range_ROS(float &cm, const float sens_char_arr[]) {
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  
  char frameid[] = "/ultrasonic_ranger";
  range_msg.header.frame_id =  frameid;
  
  range_msg.field_of_view = sens_char_arr[0];//0.01;
  range_msg.min_range = sens_char_arr[1];//0.03;
  range_msg.max_range = sens_char_arr[2];//0.4;
  
  range_msg.range = cm;
  
  range_msg.header.stamp = nh.now(); // TODO timestamp???
  
  pub_range.publish(&range_msg);
}/**/
void getandsend_ir_range_ROS(const float sens_char_arr[])
{
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  
  char frameid[] = "/ir_ranger";
  range_msg.header.frame_id =  frameid;
  
  range_msg.field_of_view = sens_char_arr[0];//0.01;
  range_msg.min_range = sens_char_arr[1];//0.03;
  range_msg.max_range = sens_char_arr[2];//0.4;
  
  range_msg.range = 5; // TODO read dist
  
  range_msg.header.stamp = nh.now();
  
  pub_range.publish(&range_msg);
}


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

void loop() {
  if (millis() > lastPing + PING_INTERVAL)
  {
    float avg_cm;
    for (byte i=0; i < MOVING_AVG; i++)
      avg_cm += moving_avg_cm[i];
    send_sonar_range_ROS(avg_cm, SONAR_CHARACTERISTICS); // ULTRASOUND = 0 (msg header file define)
    pingNextSonar();
  }
  nh.spinOnce();
}



