/* Serves as a ROS node. Receives RGB LED settings and sets their values.
 * 
 *
 */

#include <math.h>
#include <ros.h>
ros::NodeHandle nh;

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


/*********************************************************
                           SETUP
 *********************************************************/
void setup()
{
  // First set output pins:
  for (byte i=0; i<3; i++)
    pinMode(ledPinListRGB[i], OUTPUT);
  for (byte i=0; i<ledSegments; i++)
    pinMode(ledPinListSegments[i], OUTPUT);
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
  calcAndSetLED();
  delay(10);
}
