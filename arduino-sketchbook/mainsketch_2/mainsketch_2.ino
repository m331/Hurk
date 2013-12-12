/* Serves as a ROS node. Receives RGB LED settings and sets their values.
 * Sends data from distance sensors.
 *
 */

#include <math.h>
#include <ros.h>
#include <NewPing.h>


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
                        SONAR CODE
 *********************************************************/
#define SONAR_NUM     1 // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long lastPing = 0; // Holds time of last ipng
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


void checkSonar()
{
  if (millis() > lastPing + PING_INTERVAL)
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
}

/*********************************************************
                      INFRARED CODE
 *********************************************************/
// Sensor pin declarations

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

/*********************************************************
                   ROS CALLBACK FUNCTION
 *********************************************************/

// Definition of callback function that receives a status value and sets global status variable.
void statusCallback ( );

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
                           SETUP
 *********************************************************/
void setup()
{
  Serial.begin(9600);
  
}

/*********************************************************
                            LOOP
 *********************************************************/
void loop()
{
  //calcAndSetLED();
  //readIRsensor(1);
  //Serial.println("I'm OK");
  checkSonar();
  //delay(300);
  delay(loopDelay);
}
