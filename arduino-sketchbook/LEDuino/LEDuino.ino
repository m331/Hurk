/* Serves as a ROS node. Receives RGB LED settings and sets their values.
 * 
 *
 */

#include <math.h>
#include <ros.h>

/*********************************************************
                     DATA STRUCTURES
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
  ROS_ERROR // An error occurred in ROS
};

// Array for holding LEDstates with proper size. Using order of enum state.
// Entries of ledstates[] will later be accessed by e.g. ledstates[NOT_CONNECTED]
const rgbas ledstates[] = {
  { BLUE,   255, NOTHING   }, // NOT_CONNECTED RGBAS
  { BLUE,   255, BREATHING }, // CONNECTED RGBAS
  { ORANGE, 255, NOTHING   }, // MOVING
  { GREEN,  255, WAITLIGHT }, // GRABBING
  { GREEN,  255, NOTHING   }, // GRABBED
  { RED,    255, NOTHING   }, // ROS_ERROR
};

/*********************************************************
-                       CONSTANTS
 *********************************************************/

// Setting the global state variable.
state Global_State = NOT_CONNECTED; // In the beginning, it's not initialized!
// Delay in loop()
byte loopDelay = 100;
// Min brightness
byte minBrightness = 100;

// Array that holds LED pins, in the right order!
const byte ledPinList[] = {9, 10, 11};

/*********************************************************
                      LED SET FUNCTION
 *********************************************************/
// Function that gets called periodically, calculates LED params based on state and sets LEDs
void calcAndSetLED() {
  // First set standard brightness.
  float bright = ledstates[Global_State].A;
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
  
  // If special function is WAITLIGHT, use BREATHING function but with twice the period.
  if (ledstates[Global_State].S == WAITLIGHT)
  {
    // Breathing motion but half period of BREATHING.
    bright = (exp(sin(millis()/1000.0*PI)) - 0.36787944)*108.0;
  }
  
  if (minBrightness != 0)
  {
    bright = bright * (float(0xFF - minBrightness)/float(0xFF)) + (minBrightness);
  }
  
  //Serial.println(bright);
  analogWrite(13, bright);
  // This is needed for color: ledstates[Global_State].RGB
}







/*********************************************************
                   ROS CALLBACK FUNCTION
 *********************************************************/
#include <std_msgs/String.h>
//#include <std_msgs/Empty.h>

ros::NodeHandle nh;

// Definition of callback function that receives a status value and sets global status variable.
//void messageCb( const std_msgs::Empty& toggle_msg){
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
//}

void messageCb( const std_msgs::String & status_str){
  if (status_str.data[0] == 'B') //Global_State = CONNECTED;
  { digitalWrite(13, HIGH-digitalRead(13)); }
  nh.loginfo(status_str.data);
  // else, error
}

// ROOOOOOS PUBLISHER

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";




ros::Subscriber<std_msgs::String> sub("robot_status", &messageCb );


/*********************************************************
                           SETUP
 *********************************************************/
void setup()
{  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
}

/*********************************************************
                            LOOP
 *********************************************************/
void loop()
{
  //str_msg.data = hello;
  //chatter.publish( &str_msg );
  nh.spinOnce();
  //calcAndSetLED();
  delay(10);
}
