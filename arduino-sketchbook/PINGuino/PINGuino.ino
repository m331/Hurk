/* Test sketch for newping library and multiple sensors, both ultrasonic and infrared */

#include <NewPing.h>

#define SONAR_NUM     1 // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).


/********************************************
           SONAR DATA STRUCTURES
 ********************************************/

unsigned long lastPing = 0; // Holds time of last ipng
byte actSon = 0;       // Holds number of active sonar

struct sonarData {
  unsigned long timestamp[SONAR_NUM]; // Array of timestamps
  float  cm[SONAR_NUM];        // Array with distance data (in cm)
} dat; // Make dat variable


NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(9, 10, MAX_DISTANCE) // Each sensor's trigger pin, echo pin, and max distance to ping.
};



void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[actSon].check_timer())
  {
    dat.timestamp[actSon] = millis() - sonar[actSon].ping_result * 1000; // Set timestamp (beginning of SRF04 pulse)
    dat.cm[actSon] = float(sonar[actSon].ping_result) / US_ROUNDTRIP_CM; // Set data
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("HELLO");
  pinMode(8, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW); // pin 11 GND
  digitalWrite(8, HIGH); // pin 8 VCC
}

void loop() {
  if (millis() > lastPing + PING_INTERVAL)
  {
    lastPing = millis();
    
    if ( ++actSon >= SONAR_NUM) // cur.actSon also gets incremented here!
    {
      actSon = 0;
      // Here, first check IR sensors and then:
      // Send data to ROS!
      
      /**/// Test print
      for (byte i=0; i < SONAR_NUM; i++)
      { Serial.print("#:"); Serial.print(i);
        Serial.print(" t:"); Serial.print(dat.timestamp[i]);
        Serial.print(" cm:"); Serial.println(dat.cm[i]);
      }
      // dim LED if valid data
      if (dat.cm[0] > 8.0)
      {//Serial.println("PENIS");
        pinMode(13,OUTPUT);
        digitalWrite(13,LOW);}
      else digitalWrite(13,HIGH);
      /**/
    }
    
    sonar[actSon].timer_stop(); // Make sure previous timer is canceled before starting a new ping (insurance).
    dat.cm[actSon] = 0; // Make distance zero in case there's no ping echo for this sensor.
    sonar[actSon].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
  }
}



