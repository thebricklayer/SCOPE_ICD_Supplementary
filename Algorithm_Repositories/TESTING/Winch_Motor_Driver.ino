/* Connor Kerry
   stepper motor driver test code for winch - pulling TARGET
   Check PPR setting on M542T Motor Driver and match with corresponding test
   1st Test:   1 m/s test
   2nd Test: 0.1 m/s test
*/

//declare pin numbers for stp and dir
#define stp 2
#define dir 13
//#define EN  7

void setup() {
  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  //pinMode(EN, OUTPUT);
  ResetPins();
  Serial.begin(9600); //Open Serial connection for debugging
  Serial.println("Beginning test");
  delay(2000);//Wait 2 seconds for 
}

//Main loop
void loop()
{
  setup();
   while(1) {
    //runTestOne();
    runTestTwo();
    delay(60000);//Wait 1 minute after test completion
   }
}

void ResetPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(dir, HIGH);
}

void StepForward(unsigned long dt, unsigned long t0, unsigned int halfT)
{
 //Serial.println(numsteps);
 
 //each iteration should turn motor 1/8th microstep --> 3200 microsteps =  
 /*for (int i=0; i <= numsteps; i++)
 {
   digitalWrite(stp, HIGH);//drive step pin high
   delayMicroseconds(halfT);//Enough to turn at 1 m/s initially
   digitalWrite(stp, LOW);//drive low again to repeat
   delayMicroseconds(halfT);//wait 1ms
 }*/
 
 while(millis()-t0 < dt)
 {
   digitalWrite(stp, HIGH);//drive step pin high
   delayMicroseconds(halfT);//Enough to turn at 1 m/s initially
   digitalWrite(stp, LOW);//drive low again to repeat
   delayMicroseconds(halfT);//wait 1ms
 }
}

/***** 1 m/s test -- 1600 PPR ******************************************
1. Step up to 1 m/s from 0.1 m/s, accelerating at 0.3 m/s^2
2. Stay at 1 m/s for 10s
3. Either decelerate back down to 0.1 m/s at -0.3 m/s^2 or stop movement
***********************************************************************/
void runTestOne()
{ 
  // USE 1600 PPR. If you want to go up to 3200 ppr, take away the 2s
//  unsigned int halfPer;
//  unsigned long currTime, dt = 10;
//  for (unsigned int i = 86; i < halfPer*10; i++) {
//    halfPer = i;
//    currTime = millis();
//  }
  
  // 0.1 m/s
  unsigned int halfPer = 21;//microsec/pulse
  unsigned long currTime = millis();
  unsigned long dt = 1000;//runtime in ms
  StepForward(dt, currTime, halfPer);
  
  // 0.3 m/s
  halfPer = 7;//microsec/pulse
  currTime = millis();
  dt = 100;//runtime
  StepForward(dt, currTime, halfPer);
  
  // 0.5 m/s
  halfPer = 6;//microsec/pulse
  currTime = millis();
  dt = 1000;//runtime
  StepForward(dt, currTime, halfPer);
  
  // 0.7 m/s
  halfPer = 5;//microsec/pulse
  currTime = millis();
  dt = 1000;//runtime
  StepForward(dt, currTime, halfPer);
  
  // 1 m/s
  halfPer = 4;//microsec/pulse
  currTime = millis();
  dt = 1000;//runtime
  StepForward(dt, currTime, halfPer);
  
  // 0.7 m/s
  halfPer = 3;//microsec/pulse
  currTime = millis();
  dt = 1000;//runtime
  StepForward(dt, currTime, halfPer);
  
  // 0.5 m/s
  halfPer = 2;//microsec/pulse
  currTime = millis();
  dt = 10000;//runtime
  StepForward(dt, currTime, halfPer);
  
  // 0.3 m/s
  halfPer = 914;//microsec/pulse
  currTime = millis();
  dt = 1000;//runtime
  StepForward(dt, currTime, halfPer);
}

/***** 0.1 m/s test -- 12800 PPR ******************************************
1. Rotate motor for 0.1 m/s linear velocity for 140 seconds
***********************************************************************/
void runTestTwo()
{ 
  // USE 51200 PPR
  
  // 0.1 m/s
  unsigned int halfPer = 40;//microsec/pulse
  //unsigned int halfPer = 10972;//microsec/pulse - 400 ppr
  unsigned long currTime = millis();
  unsigned long dt = 150000;//runtime in ms
  StepForward(dt, currTime, halfPer);
//  halfPer = ;//microsec/pulse
//  //unsigned int halfPer = 10972;//microsec/pulse - 400 ppr
//  currTime = millis();
//  dt = 140000;//runtime in ms
//  StepForward(dt, currTime, halfPer);
}
