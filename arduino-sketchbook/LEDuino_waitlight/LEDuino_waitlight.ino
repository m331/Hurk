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
  ROS_ERROR, // An error occurred in ROS.
  ENUM_STATE_LAST_ENTRY // Last entry in enum state!
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
const byte ledPinListRGB[3]   = {9, 10, 11}; // Pins for RGB
const byte ledSegments = 4;
const byte ledPinListSegments[ledSegments] = {5, 6, 7, 8}; // Pins for different segments

/*********************************************************
                      LED SET FUNCTION
 *********************************************************/
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
    
    //Serial.println(bright);
    analogWrite(13, bright);
    // This is needed for color: ledstates[Global_State].RGB
  }
  
  // If lighting mode is waitlight, take this approach.
  else if (ledstates[Global_State].S == WAITLIGHT)
  {
    byte brightStepPerSegment = (byte(bright) - minBrightness) / (ledSegments - 1);
    for (byte i=0; i<ledSegments; i++) {
      // Calculate brightness per segment, brightness of first segment = bright, last segment = minBrightness, other segments are divided in equal steps
      byte brightness = byte(bright) - ( brightStepPerSegment * i );
      brightness = max(brightness, 0); // Constrain to >=0 just to be sure
      analogWrite(ledPinListSegments[i], brightness);
    }
  }
}







/*********************************************************
                   ROS CALLBACK FUNCTION
 *********************************************************/
//#include <std_msgs/String.h>
//#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;

// Definition of callback function that receives an empty message and blinks led. For testing.
//void messageCb( const std_msgs::Empty& toggle_msg){
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
//}

// Definition of callback function that receives a byte corresponding to a state in enum state and sets global status variable.
// Message callback that gets called when message gets received.
void messageCb( const std_msgs::Int8 & status_byte){
  if ((status_byte.data >= 0) && (status_byte.data < ENUM_STATE_LAST_ENTRY)) // Check if status_byte corresponds to a valid state
    Global_State = state(status_byte.data); // Set Global_State to the status_byte value. Global_State is a state enum. The enum must be known by the publisher.
  
  if (status_byte.data == CONNECTED)
    digitalWrite(13, HIGH-digitalRead(13)); // Blink LED
  
  //nh.loginfo(status_byte.data);
  // else, error
}

ros::Subscriber<std_msgs::Int8> sub("robot_status", &messageCb );

/* ROS hello world publisher for testing
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";
*/


/*********************************************************
                           SETUP
 *********************************************************/
void setup()
{  
  pinMode(13, OUTPUT); // For testing
  nh.initNode();
  nh.subscribe(sub);
  //nh.advertise(chatter); // For hello world testing
}

/*********************************************************
                            LOOP
 *********************************************************/
void loop()
{
  //str_msg.data = hello; // For testing
  //chatter.publish( &str_msg ); // For testing
  nh.spinOnce();
  //calcAndSetLED();
  delay(10);
}
