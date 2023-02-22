/*
    Written by Aqilla Rahman Musyaffa
*/

#include <HCSR04.h>

#define IN1 8         //terminalMotorKananBelakang  
#define IN2 9         //terminalMotorKananDepan    
#define IN3 10        //terminalMotorKiriDepan       
#define IN4 11        //terminalMotorKiriBelakang
#define trigger 3
#define echo 4

HCSR04 HC (trigger, echo);

int distance, rightDistance, leftDistance;

void setup() 
{
  Serial.begin (9600);
  
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);

  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, LOW);
}

void loop() 
{
  distance = HC.dist();
  Serial.print ("Range = ");
  Serial.println (distance);

  if (distance < 20)
  {
    pathFinding();
  }

  else
  {
    moveForward();
  }
}

void pathFinding()
{
  moveStop();
  delay (500);
  moveRight();
  delay (500);
  rightDistance = HC.dist();
  delay (500);
  moveTurnAround()
  leftDistance = HC.dist();
  delay (500);
  moveRight();
  compareDistance();
}

void moveForward()
{
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
}

void moveBackward()
{
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
}

void moveStop()
{
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, LOW);
}

void moveRight()
{
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
}

void moveLeft()
{
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
}

void moveTurnAround()
{
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  delay (1000);
}

void compareDistance()
{
  if (rightDistance > leftDistance)
  {
    moveRight();
  }

  else if (rightDistance < leftDistance)
  {
    moveLeft();
  }

  else
  {
    moveTurnAround();
  }
}
