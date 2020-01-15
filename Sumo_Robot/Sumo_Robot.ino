/*
 * code to push cans of beans around
 * 
 */

#include <Ultrasonic.h>

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

#define DISTANCE_THRESHOLD 60 //cm

Ultrasonic frontSensor(US_FRONT_TRIG, US_FRONT_ECHO, 10000UL);
Ultrasonic leftSensor(US_LEFT_TRIG, US_LEFT_ECHO, 10000UL);

int lspeed = 150;
int rspeed = 150;

bool loop_flag = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //set up pins as inputs and outputs
  pinMode(RIGHT_DIR_1, OUTPUT);
  pinMode(RIGHT_DIR_2, OUTPUT);
  pinMode(RIGHT_ENB, OUTPUT);
  pinMode(LEFT_DIR_1, OUTPUT);
  pinMode(LEFT_DIR_2, OUTPUT);
  pinMode(LEFT_ENB, OUTPUT);


  //Speed
  analogWrite(RIGHT_ENB, rspeed);
  analogWrite(LEFT_ENB, lspeed);

  // spin!
  move_robot(0, 2);
}

void loop() {

  loop_flag = true;
 
  // read ultrasonic
  

  for (int i = 0; i < 2; i++)
  {
    int distance = frontSensor.read();
    Serial.println(distance);

    if (distance > DISTANCE_THRESHOLD) 
    {
      loop_flag = false;
      move_robot(0, 2);

      
      break;
    }
    else
    {
      move_robot(0, 4);
    }

    delay(25);
  }
  
  // if distance < 60
  if (loop_flag)
  {
    Serial.println("object detected");

    //move robot anticlockwise a bit
    move_robot(100, 3);
    
    // drive forward (record no. rotations)
    move_robot(2500, 1);

    // if robot is at line
    
    // drive backwards (for same no. rotations)
    move_robot(2500, 0);

    // spin
    move_robot(0, 2);
  
  }
  else move_robot(0, 2);
  
  

  

}


void set_speed(int left, int right=-1)
{
  if (right == -1) right = left;
  
  analogWrite(RIGHT_ENB, right);
  analogWrite(LEFT_ENB, left);
}

//function to move robot
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
      break;
  }
}
