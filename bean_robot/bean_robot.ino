/**
 * bean robot, mostly from scratch
 * created on 29.01.20
 */

//includes
#include <Ultrasonic.h>
#include <Encoder.h>      //doesn't do anything yet

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

//global variables
int lspeed = 150;
int rspeed = 150;
bool loop_flag = false;
double ir_value = 0;

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


  //set initial speed
  analogWrite(RIGHT_ENB, rspeed);
  analogWrite(LEFT_ENB, lspeed);

  // spin!
  move_robot(0, DIR_CLOCKWISE);
  set_speed(100);
}

void loop() {

  //determine ultrasonic sensor distance
  int distance = frontSensor.read();
  Serial.println(distance);

  //if distance is within some range
  if (distance > 5 && distance < 60)
  {
    //stop the robot for 2s
    Serial.print("s");
    move_robot(2000, DIR_STOP);

    //set speed (so the robot moves faster when going forward)
    Serial.print("S");
    set_speed(150);

    //go forwards for 100ms
    Serial.print("f");
    move_robot(100, DIR_BACKWARD); //this is deliberately set to backwards

    //wait until robot senses line
    Serial.print("l");
    while( ir_value < 980.0 ) 
    {
      ir_value = analogRead(A0);
    }

    //reset ir value
    Serial.print("L");
    ir_value = 0;

    //go backwards for 2s
    Serial.print("b");
    move_robot(2000, DIR_FORWARD);

    //stop the robot for 2s
    Serial.print("s");
    move_robot(2000, DIR_STOP);

    //set speed (slow for spinning)
    Serial.print("S");
    set_speed(100);

    //go clockwise for 2s
    Serial.println("g");
    move_robot(2000, DIR_CLOCKWISE);
    
  }

  
}



//sets the speed of the robot's wheels.  if you only specify lspeed, rspeed will be set to the same value as lspeed
// int lspeed       the speed of the left wheel (or both)
// int rspeed       the speed of the right wheel

void set_speed(int lspeed, int rspeed=-1)
{
  if (rspeed == -1) rspeed = lspeed;
  
  analogWrite(RIGHT_ENB, rspeed);
  analogWrite(LEFT_ENB, lspeed);
}

//move the robot
// int t            time to delay (in ms) after moving the robot.  the robot will continue to move while the delay is elapsing,
//                  and will move after the delay has elapsed
// int dir          the direction to move the robot.  you can pass a number or a member of the "directions" enum
//                  number            direction
//                  0                 forward
//                  1                 backward
//                  2                 clockwise
//                  3                 anticlockwise
//                  anything else     stop the robot (the robot may coast a bit)

void move_robot(int t, int dir)
{
  switch(dir)
  {
    case 0: //forward
      //r -> clockwise
      digitalWrite(RIGHT_DIR_1, HIGH);
      digitalWrite(RIGHT_DIR_2, LOW);
      //l -> anticlockwise
      digitalWrite(LEFT_DIR_1, LOW);
      digitalWrite(LEFT_DIR_2, HIGH);
      delay(t);
      break;

   case 1: //backwards
      digitalWrite(RIGHT_DIR_1, LOW);
      digitalWrite(RIGHT_DIR_2, HIGH);
      digitalWrite(LEFT_DIR_1, HIGH);
      digitalWrite(LEFT_DIR_2, LOW);
      delay(t);
      break;

    case 2: //clockwise
      digitalWrite(RIGHT_DIR_1, LOW);
      digitalWrite(RIGHT_DIR_2, HIGH);
      digitalWrite(LEFT_DIR_1, LOW);
      digitalWrite(LEFT_DIR_2, HIGH);
      delay(t);
      break;

    case 3: //anticlockwise
      digitalWrite(RIGHT_DIR_1, HIGH);
      digitalWrite(RIGHT_DIR_2, LOW);
      digitalWrite(LEFT_DIR_1, HIGH);
      digitalWrite(LEFT_DIR_2, LOW);
      delay(t);
      break;

    default:
    case 4: //stop
      digitalWrite(RIGHT_DIR_1, LOW);
      digitalWrite(RIGHT_DIR_2, LOW);
      digitalWrite(LEFT_DIR_1, LOW);
      digitalWrite(LEFT_DIR_2, LOW);
      delay(t);
      break;
  }
}
