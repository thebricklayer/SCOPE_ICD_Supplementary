/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#define stp 4
#define dir 8

#include <TimerOne.h>
#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(2, 3);
unsigned long start = 0, prev;
long period = 703;//3515; // microsec/pulse 51200
//   avoid using pins with LEDs attached

void setup() {
  //Motor setup
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  Timer1.initialize(period);
  Timer1.setPeriod(period);
  Timer1.attachInterrupt(isr,period);

  //Encoder setup
  Serial.begin(9600);
  Serial.println("Begin Encoder Data:");
  prev = millis();
  start = millis();
}

double oldPosition  = -999.0;
double positionDeg = 0;

void loop() {
  double newPosition = (double)myEnc.read();
  
  if (millis() - prev > 50) {
    oldPosition = newPosition;
    Serial.print((double)millis()/1000.0,3); Serial.print(" ");
    Serial.print(newPosition*360/4096,3); Serial.print("\n");
    prev = millis();
  }
}

void isr()
{
  // Toggle stp every period for the duration of the test
  if (digitalRead(stp) == HIGH)
  {
    digitalWrite(stp, LOW);
  }
  else
  {
    digitalWrite(stp, HIGH);
  }
  
}
