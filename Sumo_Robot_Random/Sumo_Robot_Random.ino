/*
 * robot to push beans
 * but this one randomly selects a direction,
 * and doesn't actually detect any beans.
 * heavily adapted from https://github.com/player2point0/MineSweeper/blob/master/sketch_jan15a/sketch_jan15a.ino
 */

#include <Ultrasonic.h>
#include <Encoder.h>

#define RANDOM_DIRECTION

Ultrasonic leftSensor(A3, A2, 10000UL);
Ultrasonic frontSensor(A5, A4, 10000UL);

//Motor pins
#define L_MOTOR_ENABLE 6
#define R_MOTOR_ENABLE 5
#define L_MOTOR_1 11
#define L_MOTOR_2 9
#define R_MOTOR_1 8
#define R_MOTOR_2 7
#define MIN_MOTOR_SPEED 1
#define MAX_MOTOR_SPEED 255

#define IR_PIN A0

Encoder leftEncoder(3, 4);
Encoder rightEncoder(2, 10);
volatile long leftEncoderStore, rightEncoderStore;
volatile int rightMotorSpeed, leftMotorSpeed;
bool flag = false;

int normalSpeed = 160;
int mediumSpeed = 120;//normalSpeed / 2;//120
int slowSpeed = 70;//normalSpeed / 3;//70

void driveLeftMotor(int speed);
void driveRightMotor(int speed);
void stopMotors();

void setup() {

  Serial.begin(9600);
  //setup motors
  pinMode(L_MOTOR_1, OUTPUT);
  pinMode(L_MOTOR_2, OUTPUT);
  pinMode(R_MOTOR_1, OUTPUT);
  pinMode(R_MOTOR_2, OUTPUT);
  pinMode(L_MOTOR_ENABLE, OUTPUT);
  pinMode(R_MOTOR_ENABLE, OUTPUT);
  pinMode(IR_PIN, INPUT);
  
  //https://learn.adafruit.com/multi-tasking-the-arduino-part-2/timers
  //https://www.teachmemicro.com/arduino-timer-interrupt-tutorial/
  //https://arduino.stackexchange.com/questions/30968/how-do-interrupts-work-on-the-arduino-uno-and-similar-boards
  OCR0A = 0xAF; // set the compare register A for timer0
  TIMSK0 |= _BV(OCIE0A);  //enable the compare interrupt A for timer 0  
  
}

// interrupt service routine called when timer0 compare A interrupt flag set
//called on timer0 so every 1ms
ISR(TIMER0_COMPA_vect) 
{
  if (flag) {
    //leftMotor
    long encoderReading = leftEncoder.read();
    int measuredSpeed = abs(leftEncoderStore - encoderReading) * 15;//in rpm
    leftEncoderStore = encoderReading;
    if (measuredSpeed > leftMotorSpeed)
      digitalWrite(L_MOTOR_ENABLE, LOW);
    else
      digitalWrite(L_MOTOR_ENABLE, HIGH);
  }
  
  else
  {
    //rightMotor
    long encoderReading = rightEncoder.read();
    long measuredSpeed = abs(rightEncoderStore - encoderReading) * 15;//in rpm
    rightEncoderStore = encoderReading;
    if (measuredSpeed > rightMotorSpeed)
      digitalWrite(R_MOTOR_ENABLE, LOW);
    else
      digitalWrite(R_MOTOR_ENABLE, HIGH);  
  }

  flag = !flag;
} 

void loop() 
{
  int distanceForward = frontSensor.read();
  double ir_val = analogRead(IR_PIN);
  delay(50);

# ifdef RANDOM_DIRECTION
  if (random(50) == 1)
  {
# else
  if (distanceForward < 80)
  {
# endif /*RANDOM_DIRECTION*/

    stopMotors();
    
    //sprintForward(normalSpeed, 1000);
    while(1)
    {
      driveLeftMotor(normalSpeed);
      driveRightMotor(normalSpeed);

      if (ir_val >= 900.0) 
      {
        driveLeftMotor(-normalSpeed);
        driveRightMotor(-normalSpeed);
        break;
      }
    }
  }
  
  //else rotate
  else
  {
    driveRightMotor(-slowSpeed);
    driveLeftMotor(slowSpeed); 
    delay(20);
    stopMotors();
  }
  
}


void stopMotors()
{
  driveRightMotor(0);
  driveLeftMotor(0);
}


//MOTOR DRIVES
void driveLeftMotor(int speed)
{
  noInterrupts();
  leftMotorSpeed = abs(speed);
  interrupts();

  if (leftMotorSpeed < MIN_MOTOR_SPEED) 
  {  
    //analogWrite(L_MOTOR_ENABLE, 0);
    digitalWrite(L_MOTOR_1, LOW);
    digitalWrite(L_MOTOR_2, LOW);  
  }
  else 
  {
    //analogWrite(L_MOTOR_ENABLE, abs(speed));
    if (speed > 0)
    {
      digitalWrite(L_MOTOR_1, HIGH);
      digitalWrite(L_MOTOR_2, LOW);
    }
    else
    {
      digitalWrite(L_MOTOR_1, LOW);
      digitalWrite(L_MOTOR_2, HIGH);
    }
  }
}
void driveRightMotor(int speed)
{
  noInterrupts();
  rightMotorSpeed = 1.35 * abs(speed);
  interrupts();

  if (rightMotorSpeed < MIN_MOTOR_SPEED) 
  { 
    //analogWrite(R_MOTOR_ENABLE, 0);
    digitalWrite(R_MOTOR_1, LOW);
    digitalWrite(R_MOTOR_2, LOW);  
  }
  else 
  {
    //analogWrite(R_MOTOR_ENABLE, abs(speed));
    if (speed > 0)
    {
      digitalWrite(R_MOTOR_1, LOW);
      digitalWrite(R_MOTOR_2, HIGH);
    }
    else 
    {
      digitalWrite(R_MOTOR_1, HIGH);
      digitalWrite(R_MOTOR_2, LOW);
    }
  }
}
