/**
 * bean robot, mostly from scratch
 * created on 29.01.20
 */

//includes
#include <Ultrasonic.h>
#include <Encoder.h>

//directions for move_robot function
enum directions
{

  DIR_FORWARD = 0,
  DIR_BACKWARD,
  DIR_CLOCKWISE,
  DIR_ANTICLOCKWISE,
  DIR_STOP

};

//pin definitions
//Standard capitals because it is constant
#define RIGHT_DIR_1 8
#define RIGHT_DIR_2 7
#define RIGHT_ENB 5
#define LEFT_DIR_1 11
#define LEFT_DIR_2 9
#define LEFT_ENB 6

#define US_FRONT_TRIG A5
#define US_FRONT_ECHO A4

#define US_LEFT_TRIG A3
#define US_LEFT_ECHO A2

#define IR_PIN A0;

//object construction
Ultrasonic frontSensor(US_FRONT_TRIG, US_FRONT_ECHO, 10000UL);
Ultrasonic leftSensor(US_LEFT_TRIG, US_LEFT_ECHO, 10000UL);

Encoder leftEncoder(3, 4);
Encoder rightEncoder(2, 10);

//global variables
int lspeed = 150;
int rspeed = 150;
bool loop_flag = false;
double ir_value = 0;
bool int_motor = false;
bool pushing = false;
volatile long leftEncoderStore = 0, rightEncoderStore = 0;
volatile int rightMotorSpeed = 0, leftMotorSpeed = 0;

int motor_speed_spin = 40;
int motor_speed_drive = 120;

//function prototypes
void move_robot(int t, int dir);
void driveLeftMotor(int);
void driveRighttMotor(int);

// interrupt service routine called when timer0 compare A interrupt flag set
//called on timer0 so every 1ms
ISR(TIMER0_COMPA_vect) 
{

  if (int_motor) {
    //leftMotor
    long encoderReading = leftEncoder.read();
    int measuredSpeed = abs(leftEncoderStore - encoderReading) * 15;//in rpm
    leftEncoderStore = encoderReading;
    if (measuredSpeed > leftMotorSpeed)
      digitalWrite(LEFT_ENB, LOW);
    else
      digitalWrite(LEFT_ENB, HIGH);
  }
  
  else
  {
    //rightMotor
    long encoderReading = rightEncoder.read();
    long measuredSpeed = abs(rightEncoderStore - encoderReading) * 15;//in rpm
    rightEncoderStore = encoderReading;
    if (measuredSpeed > rightMotorSpeed)
      digitalWrite(RIGHT_ENB, LOW);
    else
      digitalWrite(RIGHT_ENB, HIGH);
  }

  int_motor = !int_motor;
} 

//setup function
void setup() {

  //start serial
  Serial.begin(9600);

  //set up pins as inputs and outputs
  pinMode(RIGHT_DIR_1, OUTPUT);
  pinMode(RIGHT_DIR_2, OUTPUT);
  pinMode(RIGHT_ENB, OUTPUT);
  pinMode(LEFT_DIR_1, OUTPUT);
  pinMode(LEFT_DIR_2, OUTPUT);
  pinMode(LEFT_ENB, OUTPUT);
  pinMode(A0, INPUT);

  //configute avr timer
  //stolen from https://github.com/player2point0/MineSweeper/blob/master/sketch_jan15a/sketch_jan15a.ino
  //https://learn.adafruit.com/multi-tasking-the-arduino-part-2/timers
  //https://www.teachmemicro.com/arduino-timer-interrupt-tutorial/
  //https://arduino.stackexchange.com/questions/30968/how-do-interrupts-work-on-the-arduino-uno-and-similar-boards
  OCR0A = 0xAF; // set the compare register A for timer0
  TIMSK0 |= _BV(OCIE0A);  //enable the compare interrupt A for timer 0  

  // spin!
    driveLeftMotor(motor_speed_spin);
    driveRightMotor(-motor_speed_spin);
    delay(100);
//  move_robot(0, DIR_CLOCKWISE);
}

void loop() {
 
  //determine ultrasonic sensor distance
  int distance = frontSensor.read();
  Serial.println(distance);

  //if distance is within some range
  if (distance > 5 && distance < 60)
  {
    //record start encoder position
    long start_encoder = leftEncoderStore;
    
    //stop the robot for 0.5s
    //atm the robot will drift backwards a bit, that's fine
    Serial.println("stop robot");
    driveLeftMotor(0);
    driveRightMotor(0);
    delay(500);
//  move_robot(2000, DIR_STOP);

    //go forwards for 100ms
    Serial.println("forward");
    driveLeftMotor(motor_speed_drive);
    driveRightMotor(motor_speed_drive);
    delay(100);
//  move_robot(100, DIR_BACKWARD); //this is deliberately set to backwards

    //wait until robot senses line
    Serial.println("wait for line");
    while( ir_value < 980.0 ) 
    {
      ir_value = analogRead(A0);
    }

    //reset ir value
    Serial.println("found line");
    ir_value = 0;

    //go backwards for 3s
    Serial.println("go backwards");
    driveLeftMotor(-motor_speed_drive);
    driveRightMotor(-motor_speed_drive);
    delay(3000);

    //store encoder position at line
    long end_encoder = leftEncoderStore;

    //go backwards for the same distance as the robot went forwards
    /*
    while(1)
    {
      break;
      Serial.print("distance: ");
      Serial.print(end_encoder  - start_encoder);
      Serial.print("\t\t\telapsed: ");
      Serial.println(leftEncoderStore - end_encoder);

      //todo: fix this
      if (end_encoder  - start_encoder < leftEncoderStore - end_encoder) break;
    }
    */

    //stop the robot for 0.5s
    Serial.println("stop robot");
    driveLeftMotor(0);
    driveRightMotor(0);
    delay(500);
//  move_robot(2000, DIR_STOP);

    //go clockwise for 0.1s
    Serial.println("go clockwise");
    driveLeftMotor(motor_speed_spin);
    driveRightMotor(-motor_speed_spin);
    delay(100);
//  move_robot(2000, DIR_CLOCKWISE);
  }

  
}


//move the robot
// int dir          the direction to move the robot.  you can pass a number or a member of the "directions" enum
//                  number            direction
//                  0                 forward
//                  1                 backward
//                  2                 clockwise
//                  3                 anticlockwise
//                  anything else     stop the robot (the robot may coast a bit)
// int t            time to delay (in ms) after moving the robot.  the robot will continue to move while the delay is elapsing,
//                  and will move after the delay has elapsed
// int speed        the speed that the robot will move

void move_robot(int dir, int t, int speed)
{
  switch(dir)
  {
    case 0: //forward
      driveLeftMotor(speed);
      driveRightMotor(speed);
      break;

   case 1: //backwards
      driveLeftMotor(-speed);
      driveRightMotor(-speed);
      break;

    case 2: //clockwise
      driveLeftMotor(speed);
      driveRightMotor(-speed);
      break;

    case 3: //anticlockwise
      driveLeftMotor(speed);
      driveRightMotor(-speed);
      break;

    default:
    case 4: //stop
      driveLeftMotor(0);
      driveRightMotor(0);
      
      digitalWrite(RIGHT_DIR_1, LOW);
      digitalWrite(RIGHT_DIR_2, LOW);
      digitalWrite(LEFT_DIR_1, LOW);
      digitalWrite(LEFT_DIR_2, LOW);
      break;
  }
  delay(t);
}

//MOTOR FUNCTIONS
//https://github.com/kmclaughlin/Robot_Platform/blob/master/Firmware/Advanced_Movement/Bare_bones/Bare_bones.ino
void driveLeftMotor(int speed){
  leftMotorSpeed = abs(speed);
  if (speed > 0){
    digitalWrite(LEFT_DIR_1, HIGH);
    digitalWrite(LEFT_DIR_2, LOW);
  }
  else{
    digitalWrite(LEFT_DIR_1, LOW);
    digitalWrite(LEFT_DIR_2, HIGH);
  }
}

void driveRightMotor(int speed){
  rightMotorSpeed = abs(speed);
  if (speed > 0){
    digitalWrite(RIGHT_DIR_1, LOW);
    digitalWrite(RIGHT_DIR_2, HIGH);
  }
  else {
    digitalWrite(RIGHT_DIR_1, HIGH);
    digitalWrite(RIGHT_DIR_2, LOW);
  }
}
