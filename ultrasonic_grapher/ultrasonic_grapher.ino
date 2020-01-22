
/*
 * _can_ the ultrasonic sensor detect curved cans?  haha
 */

#include <Ultrasonic.h>

#define US_FRONT_TRIG A5
#define US_FRONT_ECHO A4

#define US_LEFT_TRIG A3
#define US_LEFT_ECHO A2

Ultrasonic frontSensor(US_FRONT_TRIG, US_FRONT_ECHO, 30000UL);

unsigned int thing = 0;
int circularBuffer[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int bufferIndex = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  circularBuffer[bufferIndex] = frontSensor.read();
  bufferIndex++;
  if(bufferIndex > 9) bufferIndex = 0;

  for (int i = 0; i < 10; i++)
  {
    thing += circularBuffer[i];
  }
  delay(50);
  
  thing /= 10;
  Serial.println(thing);
  thing = 0;
}
