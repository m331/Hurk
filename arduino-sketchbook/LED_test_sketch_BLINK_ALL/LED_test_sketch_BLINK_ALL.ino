/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int leds[] = {3,9,10,11};

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  for (byte i=0; i<4; i++) {
    pinMode(leds[i], OUTPUT);
  }
  pinMode(12, OUTPUT); pinMode(13,OUTPUT);
  analogWrite(12, 0xFE); analogWrite(13,0xFE);
}

unsigned long period = 4000;
unsigned long T = 255;
// the loop routine runs over and over again forever:
void loop() {
  for (byte i=0; i<4; i++) {
    analogWrite(leds[i], 0xFE);   // turn the LED on (HIGH is the voltage level)
    //delay(800);               // wait for a second
    //analogWrite(leds[i], 0x04);    // turn the LED off by making the voltage LOW
    //delay(800*(i+1));               // wait for a second
  }
  analogWrite(12, (millis()%period/(2000/0xFF)));
  analogWrite(13, (millis()%period/(2000/0xFF)));
}
