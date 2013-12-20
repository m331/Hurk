/* Test sketch for newping library and multiple sensors, both ultrasonic and infrared */

#include "RunningAverage.h"
#include <math.h>
#include <NewPing.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);

/*********************************************************
                     LED DATA STRUCTURES
 *********************************************************/
// Struct for defining RGB values
struct rgb {
  byte R;  byte G;  byte B;
};
// Struct with RGBA value + S byte which contains a special state descriptor.
struct rgbas {
  rgb RGB;
  byte A;
  byte S;
};

// Defining colors
const rgb BLUE = {0,0,255};
const rgb GREEN = {0,255,0};
const rgb ORANGE = {255,165,0};
const rgb RED = {255,0,0};
const rgb WHITE = {0xFF,0xFF,0xFF};

// Enum to describe special states. For code readability.
enum S {
  NOTHING, // No special state
  BLINKING, // Blinking now and then
  WAITLIGHT, // Turn round
  BREATHING // Breathing light
};

// Enum to describe defined states. For code readability.
enum state {
  NOT_CONNECTED, // Arduino not connected to ROS
  CONNECTED, // Connected and in rest
  MOVING, // Moving around
  GRABBING, // Picking something up
  GRABBED, // Grabbed something
  ROS_ERROR, // An error occurred in ROS.
  ENUM_STATE_LAST_ENTRY // Last entry in enum state!
};

// Array for holding LEDstates with proper size. Using order of enum state.
// Entries of ledstates[] will later be accessed by e.g. ledstates[NOT_CONNECTED]
const rgbas ledstates[] = {
  { BLUE,   255, NOTHING   }, // NOT_CONNECTED RGBAS
  { BLUE,   255, BREATHING }, // CONNECTED RGBAS
  { ORANGE, 255, NOTHING   }, // MOVING
  { WHITE,  255, WAITLIGHT }, // GRABBING
  { GREEN,  255, NOTHING   }, // GRABBED
  { RED,    255, NOTHING   }, // ROS_ERROR
};

/*********************************************************
-                      LED CONSTANTS
 *********************************************************/

// Setting the global state variable.
state Global_State = GRABBING; // In the beginning, it's not initialized!
// Delay in loop()
byte loopDelay = 100;
// Min brightness
byte minBrightness = 4;

// Array that holds LED pins, in the right order!
const byte ledPinListRGB[3]   = {12, 13, 13}; // Pins for RGB in order R, G, B
const byte ledSegments = 4;
const byte ledPinListSegments[ledSegments] = {3, 9, 10, 11}; // Pins for different segments

/*********************************************************
                      LED SET FUNCTION
 *********************************************************/
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// Function that gets called periodically, calculates LED params based on state and sets LEDs
void calcAndSetLED() {
  // First set standard brightness.
  float bright = ledstates[Global_State].A;
  
  // Waitlight needs a fundamentally different approach. So first define the approach for all other lighting modes.
  if (ledstates[Global_State].S != WAITLIGHT)
  {
    // If special function is BREATHING, calculate e^sin(x) shape.
    if (ledstates[Global_State].S == BREATHING)
    {
      bright = (exp(sin(millis()/2000.0*PI)) - 0.36787944)*108.0;
    }
    // If special function is BLINKING, blink with period T.
    if (ledstates[Global_State].S == BLINKING)
    {
      unsigned long T = 500;
      ((millis() % T) < T/2) ? bright=0xFF : bright=0x00 ;
    }
    
    if (minBrightness != 0)
    {
      bright = bright * (float(0xFF - minBrightness)/float(0xFF)) + (minBrightness);
    }
    
    for (byte i=0; i<ledSegments; i++) {
      // Calculate brightness per segment, brightness of first segment = bright, last segment = minBrightness, other segments are divided in equal steps
      byte brightness = max(bright, 0); // Constrain to >=0 just to be sure
      analogWrite(ledPinListSegments[i], brightness);
    }
  }
  
  // If lighting mode is waitlight, take this approach. Simulates round rotating waiting symbol.
  else if (ledstates[Global_State].S == WAITLIGHT)
  {
    //nh.loginfo("I'm here!");
    // float brightRefPerSegment
    float bRPS[ledSegments] = {0xFF, 70, 26, 1};
    unsigned int T = 600; // Time for one full rotation in milliseconds
    float timePosInLoop = float(millis()%T) * float(ledSegments) / float(T); // Time position in loop, from 0 to ledSegments
    
    for (byte i=0; i<ledSegments; i++) {
      //timePosInLoop = .98; Testing
      float posInLoop = fmod((float(i) + timePosInLoop), float(ledSegments));
      float brightness = 0;
      if (posInLoop < 1) {
        brightness = mapf(posInLoop, 0.,1., bRPS[0], bRPS[1]);
      }
      else if (posInLoop < 2) {
        brightness = mapf(posInLoop, 1.,2., bRPS[1], bRPS[2]);
      }
      else if (posInLoop < 3) {
        brightness = mapf(posInLoop, 2.,3., bRPS[2], bRPS[3]);
      }
      else if (posInLoop <4) {
        brightness = mapf(posInLoop, 3.,4., bRPS[3], bRPS[0]);
      }
      analogWrite(ledPinListSegments[i], brightness);
    }
    /* FOR TESTING
    analogWrite(ledPinListSegments[0], 0xFF);
    analogWrite(ledPinListSegments[1], 0xFF-220);
    analogWrite(ledPinListSegments[2], 0xFF-245);
    analogWrite(ledPinListSegments[3], 0xFF-0xFF);
    // END TESTING /**/
  }
  
  // Now set color on RGB pins:
  // This is needed for color: ledstates[Global_State].RGB
  analogWrite(ledPinListRGB[0], ledstates[Global_State].RGB.R);
  analogWrite(ledPinListRGB[1], ledstates[Global_State].RGB.G);
  //analogWrite(ledPinListRGB[2], ledstates[Global_State].RGB.B);
}


/*********************************************************
                LED ROS CALLBACK FUNCTION
 *********************************************************/
#include <std_msgs/Int8.h>

// Definition of callback function that receives a byte corresponding to a state in enum state and sets global status variable.
// Message callback that gets called when message gets received.
void messageCb( const std_msgs::Int8 & status_byte){
  if ((status_byte.data >= 0) && (status_byte.data < ENUM_STATE_LAST_ENTRY)) // Check if status_byte corresponds to a valid state
    Global_State = state(status_byte.data); // Set Global_State to the status_byte value. Global_State is a state enum. The enum must be known by the publisher.
}

ros::Subscriber<std_msgs::Int8> sub("robot_status", &messageCb );



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
      actSon = 0;
    
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
    // First set output pins:
  for (byte i=0; i<3; i++)
    pinMode(ledPinListRGB[i], OUTPUT);
  for (byte i=0; i<ledSegments; i++)
    pinMode(ledPinListSegments[i], OUTPUT);

  nh.initNode();
  nh.advertise(pub_range);
  nh.subscribe(sub);
  
  myRA[actSon].clear(); // explicitly start clean
}

void loop() {
  if (millis() > lastPing + PING_INTERVAL)
  {
    send_sonar_range_ROS( myRA[actSon].getAverage() , SONAR_CHARACTERISTICS , actSon );
    pingNextSonar();
  }
  
  calcAndSetLED();
  
  nh.spinOnce();
}



