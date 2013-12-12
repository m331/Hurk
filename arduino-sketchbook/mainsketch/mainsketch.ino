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
           ADVANCED SONAR CODE (NOT COMPLETED)
 *********************************************************/
#define TOTAL_SONARS   1 // Total number of sonars
// Pin where all echo pins are connected to.
#define ECHO_INTPIN  1 // NOTE: 0 is int0 is pin 2, 1 is int1 is pin 3
#define MAX_DISTANCE 200  // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
const byte SONAR_PIN_ARRAY[TOTAL_SONARS] = {4}; // Holds all trig pins

unsigned int sonar_data[TOTAL_SONARS];

// Global variable to hold the active sonar number.
int active_sonar = 0; // Is <= 0 when new sonar can be triggered, is >0 when sonar is active
unsigned long active_sonar_trigtime = 0;
unsigned long active_sonar_risetime = 0;

void sonarRisingInterrupt() {
  active_sonar_risetime = millis();
  detachInterrupt(ECHO_INTPIN);
  attachInterrupt(ECHO_INTPIN, sonarFallingInterrupt, FALLING);
  Serial.println("GREAT SUCCESS");
}

void sonarFallingInterrupt() { // attachInterrupt(...) must be called in setup()!
  sonar_data[active_sonar] = millis() - active_sonar_risetime;
  active_sonar = - active_sonar; // Invert active_sonar
  detachInterrupt(ECHO_INTPIN);
  Serial.println("GREAT SUCCESS");
}

void trigSonar()
{
  if ( millis() > (active_sonar_trigtime+PING_INTERVAL) )
  {
    if ( active_sonar > 0 ) // If sonar is still active (>0), error!
    { Serial.println("ERROR in trigSonar: still active sonar!"); }
    byte n = (-active_sonar) + 1; // Make active_sonar positive, then increment
    if (n >= TOTAL_SONARS) n = 0; // If n >= SONAR_PIN_ARRAY, start with first sensor again.
    active_sonar_trigtime = millis();
    digitalWrite(SONAR_PIN_ARRAY[n], HIGH);
    delayMicroseconds(100); // Delay for 10us to trigger sensor
    digitalWrite(SONAR_PIN_ARRAY[n], LOW);
    
    attachInterrupt(ECHO_INTPIN, sonarRisingInterrupt, RISING);
    
    Serial.print("SONAR: ["); Serial.print(sonar_data[0]); Serial.println("]");
  }
}/**/

/*********************************************************
                    SIMPLE SONAR CODE (bad)
 *********************************************************
#define TRIGGER_PIN  9  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     10  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 120 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define TOTAL_SONARS   1 // Total number of sonars

NewPing sonar[SONAR_NUM] = { // Sensor object array.
  NewPing(9, 10, MAX_DISTANCE) // Each sensor's trigger pin, echo pin, and max distance to ping.
};

unsigned int cm[SONAR_NUM]; // Where the ping distances are stored.

void readAllSonars()
{
  for (byte i=0; i<TOTAL_SONARS; i++)
  {
      cm[i] = sonar.ping() / US_ROUNDTRIP_CM ; // Read sonar ping, then convert to cm
  }

}*/

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
  trigSonar();
  delay(300);
  delay(loopDelay);
}
