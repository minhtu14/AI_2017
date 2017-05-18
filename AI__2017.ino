#include "SharpIR.h"
#include "BluetoothDevice.h"
#include <SoftwareSerial.h>
//Distance Sensor Left
#define DS_TRIGGER_1_PIN 	4
#define DS_ECHO_1_PIN		5

//Distance Sensor Right
#define DS_TRIGGER_2_PIN 	12
#define DS_ECHO_2_PIN 		13

//Distance Sensor Sharp IR Left
#define   IR_PIN_LEFT     A0

//Distance Sensor Sharp IR  Right
#define   IR_PIN_RIGHT    A1
#define   MODEL_IR        1080


//Distance Sensor Center
#define DS_TRIGGER_3_PIN 	A2
#define DS_ECHO_3_PIN 		A3


//Motor LEFT
#define MOT_ENABLE_1_PIN 	5
#define MOT_IN_1_PIN 		7
#define MOT_IN_2_PIN 		8


//Motor RIGHT
#define MOT_ENABLE_2_PIN 	6
#define MOT_IN_3_PIN 		10
#define MOT_IN_4_PIN 		11


//Speed Sensor
#define SS_D0_PIN 			2
#define SS_D1_PIN 			3

//Speed limit
#define MAX_SPEED     		200 //200
#define MID_SPEED     		160 //160
#define LOW_SPEED     		140 //140
#define MIN_SPEED     		100 //100


//Collision Distance
#define EX_LOW_DISTANCE 	10
#define LOWER_DISTANCE     	18 //18
#define LOW_DISTANCE 		25
#define MED_DISTANCE 		60 //60
#define HIGH_DISTANCE 		100



//Direction
#define DIR_12				12
#define DIR_2				22
#define DIR_4				44
#define DIR_6				66
#define DIR_8				88
#define DIR_10				10
#define STOP				-1


//Change Direction Step
#define TURN_COLLISION		1
#define TURN_STEP        	3
#define MOVE_BACK_STEP		9

//Loop step
#define TIME_STEP			50

//LED
#define LED_1_PIN			13
#define LED_2_PIN			12
#define LED_3_PIN			A5


//Macro
#define HAVE_COLLISION(x, d)  (x < d)

//Balance
#define BALANCE_RATIO 0.15 //

//// BLUETOOTH DEVICE.
#define   BLT_RX_PIN                13
#define   BLT_TX_PIN                12
BluetoothDevice                     blueDevice;
String DIR_LOG = "";
int distanceSensorLeft, distanceSensorRight, distanceSensorCenter, speedSensorLeft, speedSensorRight;
int speedMT1, speedMT2;
int timer, direction, nextDirection, prevDirection, changeDirectionStep;
int turn, delaystep, countdir = 0;
SharpIR irLeft(IR_PIN_LEFT, MODEL_IR);
SharpIR irRight(IR_PIN_RIGHT, MODEL_IR);

// put your setup code here, to run once:
void setup()
{
  // initialize serial communication:
  Serial.begin(9600);

  // Setup Debug With Bluetooth.
  blueDevice.SetPinBLT(BLT_RX_PIN, BLT_TX_PIN);

  //Setup distance sensor
  SetupDistanceSensor();

  //Setup motor
  SetupMotor();

  //Setup speed sensor
  SetupSpeedSensor(0);

  //setup timer
  timer = 0;

  //setup defaul direction
  direction = DIR_12;
  changeDirectionStep = 0;
  turn = TURN_STEP;

  //anti stuck
  delaystep = 0;
  countdir = 0;
}


void loop()
{


  if (timer % 10 == 0)
  {
    SetupSpeedSensor(1);
  }
  // put your main code here, to run repeatedly:

  distanceSensorLeft = irLeft.distance();
  distanceSensorRight = irRight.distance();
  distanceSensorCenter = GetDistanceForSensorUltraonic(DS_TRIGGER_3_PIN, DS_ECHO_3_PIN);

  if (distanceSensorLeft == 0)
  {
    distanceSensorLeft = HIGH_DISTANCE;
  }
  if (distanceSensorRight == 0)
  {
    distanceSensorRight = HIGH_DISTANCE;
  }

  switch (direction)
  {
    case DIR_12:
      countdir ++;

      if (HAVE_COLLISION(distanceSensorCenter, LOW_DISTANCE))
      {
        speedMT1 = MIN_SPEED;
        speedMT2 = MIN_SPEED;

        if (HAVE_COLLISION(distanceSensorLeft, LOWER_DISTANCE) && HAVE_COLLISION(distanceSensorRight, LOWER_DISTANCE))
        {
          direction = STOP;
          nextDirection = DIR_6; // turn back
          turn = MOVE_BACK_STEP;
        }
        else
        {
          if (distanceSensorLeft > distanceSensorRight)
          {
            direction = DIR_10;
            turn = TURN_STEP;
          }
          else
          {
            direction = DIR_2;
            turn = TURN_STEP;
          }
        }
      }
      else if (HAVE_COLLISION(distanceSensorCenter, MED_DISTANCE))
      {
        if (speedMT1 > LOW_SPEED || speedMT2 > LOW_SPEED)
        {
          speedMT1 = (1 - 0.2) * speedMT1;
          speedMT2 = speedMT1;
        }

      }
      else // > MED_DISTANCE
      {
        speedMT1 = MAX_SPEED;
        speedMT2 = MAX_SPEED;
      }

      if (HAVE_COLLISION(distanceSensorLeft, LOWER_DISTANCE) && !HAVE_COLLISION(distanceSensorRight, EX_LOW_DISTANCE ) )
      {
        //if (HAVE_COLLISION(distanceSensorLeft, LOWER_DISTANCE) )
        {
          speedMT1 = MID_SPEED;
          speedMT2 = MID_SPEED;
          direction = DIR_2; // turn right ( i guess)
          turn = TURN_COLLISION;
        }
      }

      if (HAVE_COLLISION(distanceSensorRight, LOWER_DISTANCE) && !HAVE_COLLISION(distanceSensorLeft, EX_LOW_DISTANCE ) )
      {
        // if (HAVE_COLLISION(distanceSensorRight, LOWER_DISTANCE) )
        {
          speedMT1 = MID_SPEED;
          speedMT2 = MID_SPEED;
          direction = DIR_10; // turn left ( i guess)
          turn = TURN_COLLISION;
        }
      }

      if (distanceSensorLeft > LOW_DISTANCE && distanceSensorRight > LOW_DISTANCE)
      {
        if (distanceSensorLeft > distanceSensorRight)
        {
          direction = DIR_10; // turn right ( i guess)
          turn = TURN_STEP;
        }
        else
        {
          direction = DIR_2; // turn right ( i guess)
          turn = TURN_STEP;
        }

      }
      break;


    case DIR_6: // turn back
      speedMT1 = MIN_SPEED;
      speedMT2 = MIN_SPEED;

      changeDirectionStep++;
      if (changeDirectionStep >= turn || (!HAVE_COLLISION(distanceSensorCenter, LOW_DISTANCE)))
      {

        // blueDevice.SentToBluetoothDevice("___________________ DIR_6 turn back              ");
        //blueDevice.SentToBluetoothDevice(88888);
        DIR_LOG = "DIR_6";
        changeDirectionStep = 0;
        direction = STOP;
        nextDirection = DIR_12;
      }

      countdir = 0;
      break;

    case DIR_2:
      //      speedMT1 = MID_SPEED;
      //      speedMT2 = MID_SPEED;
      if (speedMT1 > speedMT2)
        speedMT2 = speedMT1;
      else
        speedMT1 = speedMT2;

      changeDirectionStep++;
      if (changeDirectionStep >= turn || (!HAVE_COLLISION(distanceSensorCenter, 50)))
      {
        DIR_LOG = "DIR_2";

        changeDirectionStep = 0;
        direction = DIR_12;
      }

      if (countdir < 2)
      {
        delaystep ++;
        if (delaystep == 7)
        {
          direction = STOP;
          nextDirection = DIR_6;
          turn = MOVE_BACK_STEP;
          delaystep = 0;
        }
      }
      else
      {
        delaystep = 0;
      }

      countdir = 0;
      break;


    case DIR_10:
      //      speedMT1 = MID_SPEED;
      //      speedMT2 = MID_SPEED;
      if (speedMT1 > speedMT2)
        speedMT2 = speedMT1;
      else
        speedMT1 = speedMT2;
      changeDirectionStep++;
      if (changeDirectionStep >= turn || (!HAVE_COLLISION(distanceSensorCenter, 50)))
      {
        DIR_LOG = "DIR_10";

        changeDirectionStep = 0;
        direction = DIR_12;
      }

      if (countdir < 2)
      {
        delaystep ++;
        if (delaystep == 7)
        {
          direction = STOP;
          nextDirection = DIR_6;
          turn = MOVE_BACK_STEP;
          delaystep = 0;
        }
      }
      else
      {
        delaystep = 0;
      }

      countdir = 0;
      break;
    case STOP:
      DIR_LOG = "STOP";

      changeDirectionStep++;
      if (changeDirectionStep >= 1)
      {
        changeDirectionStep = 0;
        direction = nextDirection;
      }
      break;
  }
  prevDirection = direction;
  speedBalance(direction, speedMT1, speedMT2, distanceSensorLeft, distanceSensorRight, 0);
  RunCar(direction, speedMT1, speedMT2, distanceSensorLeft, distanceSensorRight);
  String temp = "ML:";
  temp += speedMT1;
  temp += " :";
  temp += "MR:";
  temp += speedMT2;
  temp += ": ";
  temp += "SSL:";
  temp += distanceSensorLeft;
  temp += ": ";
  temp += "SSR:";
  temp += distanceSensorRight;
  temp += ": ";
  temp += "SSH:";
  temp += distanceSensorCenter;
  temp += ": ";
  temp += direction;
  blueDevice.SentToBluetoothDevice(temp);
  delay(TIME_STEP);
  if (timer % 10 == 0)
  {
    SetupSpeedSensor(0);
  }

  if (++timer == 1000)
  {
    timer = 0;
  }
}

void RunCar(int dir, int speedM1, int speedM2, int distanceSensorL, int distanceSensorR)
{
  if (dir == DIR_12) // move ahead
  {
    RunMotor(1, speedM1, MOT_ENABLE_1_PIN, MOT_IN_1_PIN, MOT_IN_2_PIN);
    RunMotor(1, speedM2, MOT_ENABLE_2_PIN, MOT_IN_3_PIN, MOT_IN_4_PIN);
  }
  if (dir == DIR_10) // turn left
  {
    RunMotor(-1, speedM1, MOT_ENABLE_1_PIN, MOT_IN_1_PIN, MOT_IN_2_PIN);
    RunMotor(1, speedM2, MOT_ENABLE_2_PIN, MOT_IN_3_PIN, MOT_IN_4_PIN);
  }
  if (dir == DIR_2) // turn right
  {
    RunMotor(1, speedM1, MOT_ENABLE_1_PIN, MOT_IN_1_PIN, MOT_IN_2_PIN);
    RunMotor(-1, speedM2, MOT_ENABLE_2_PIN, MOT_IN_3_PIN, MOT_IN_4_PIN);
  }
  if (dir == DIR_4)
  {
    RunMotor(-1, speedM1, MOT_ENABLE_1_PIN, MOT_IN_1_PIN, MOT_IN_2_PIN);
    RunMotor(0, speedM2, MOT_ENABLE_2_PIN, MOT_IN_3_PIN, MOT_IN_4_PIN);
  }
  if (dir == DIR_8)
  {
    RunMotor(0, speedM1, MOT_ENABLE_1_PIN, MOT_IN_1_PIN, MOT_IN_2_PIN);
    RunMotor(-1, speedM2, MOT_ENABLE_2_PIN, MOT_IN_3_PIN, MOT_IN_4_PIN);
  }
  if (dir == DIR_6)
  {
    //    if (HAVE_COLLISION(distanceSensorL, EX_LOW_DISTANCE))
    //    {
    //      RunMotor(0, speedM1, MOT_ENABLE_1_PIN, MOT_IN_1_PIN, MOT_IN_2_PIN);
    //      RunMotor(-1, speedM2, MOT_ENABLE_2_PIN, MOT_IN_3_PIN, MOT_IN_4_PIN);
    //    }
    //
    //    else if (HAVE_COLLISION(distanceSensorR, EX_LOW_DISTANCE))
    //    {
    //      RunMotor(-1, speedM1, MOT_ENABLE_1_PIN, MOT_IN_1_PIN, MOT_IN_2_PIN);
    //      RunMotor(0, speedM2, MOT_ENABLE_2_PIN, MOT_IN_3_PIN, MOT_IN_4_PIN);
    //    }
    //    else
    //    {
    RunMotor(0, speedM1, MOT_ENABLE_1_PIN, MOT_IN_1_PIN, MOT_IN_2_PIN);
    RunMotor(0, speedM2, MOT_ENABLE_2_PIN, MOT_IN_3_PIN, MOT_IN_4_PIN);
    //    }
  }
  if (dir == STOP)
  {
    RunMotor(-1, speedM1, MOT_ENABLE_1_PIN, MOT_IN_1_PIN, MOT_IN_2_PIN);
    RunMotor(-1, speedM2, MOT_ENABLE_2_PIN, MOT_IN_3_PIN, MOT_IN_4_PIN);
  }
}

void speedBalance(int dir, int &speed1, int &speed2, long distance1, long distance2, long delta)
{
  float ratio = BALANCE_RATIO;
  //delta = EX_LOW_DISTANCE; // test
  if (distance1  > MED_DISTANCE && distance2 > MED_DISTANCE)
  {
    //Do no thing

  }
  else if (distance1 > distance2 + delta)
  {
    if (dir == STOP)
    {
      if (speed2 * (1 + ratio) <= 255)
        speed2 = speed2 * (1 + ratio);
      else
        speed1 = speed1 * (1 - ratio);

      if (speed1 < MIN_SPEED || speed2 < MIN_SPEED)
      {
        speed1 = LOW_SPEED;
        speed2 = LOW_SPEED * (1 + ratio);

      }
    }
    else
    {
      if (speed1 * (1 + ratio) <= 255)
        speed1 = speed1 * (1 + ratio);
      else
        speed2 = speed2 * (1 - ratio);

      if (speed1 < MIN_SPEED || speed2 < MIN_SPEED)
      {
        speed1 = LOW_SPEED * (1 + ratio);
        speed2 = LOW_SPEED;
      }
    }
  }
  else if (distance1 < distance2 + delta)
  {
    if (dir == STOP)
    {
      if (speed1 * (1 + ratio) <= 255)
        speed1 = speed1 * (1 + ratio);
      else
        speed2 = speed2 * (1 - ratio);

      if (speed1 < MIN_SPEED || speed2 < MIN_SPEED)
      {
        speed1 = LOW_SPEED * (1 + ratio);
        speed2 = LOW_SPEED;
      }

    }
    else
    {
      if (speed2 * (1 + ratio) <= 255)
        speed2 = speed2 * (1 + ratio);
      else
        speed1 = speed1 * (1 - ratio);

      if (speed1 < MIN_SPEED || speed2 < MIN_SPEED)
      {
        speed1 = LOW_SPEED;
        speed2 = LOW_SPEED * (1 + ratio);

      }
    }
  }
}
void SetupSpeedSensor(int stop)
{
  if (stop == 1)
  {
    detachInterrupt(digitalPinToInterrupt(SS_D0_PIN));
    detachInterrupt(digitalPinToInterrupt(SS_D1_PIN));
  }
  else
  {
    pinMode(SS_D0_PIN, INPUT_PULLUP);
    pinMode(SS_D1_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(SS_D0_PIN), SpeedSensorLeft, RISING );
    attachInterrupt(digitalPinToInterrupt(SS_D1_PIN), SpeedSensorRight, RISING );

    speedSensorLeft = 0;
    speedSensorRight = 0;
  }
}


void SpeedSensorLeft()
{
  speedSensorLeft++;
}


void SpeedSensorRight()
{
  speedSensorRight++;
}

void RunMotor(int dir, int speed, int En, int In1, int In2)
{
  if (dir == 1)
  {
    // turn on motor A
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);

    // set speed to 200 out of possible range 0~255
    analogWrite(En, speed);
  }
  else if (dir == 0)
  {
    // turn on motor A
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);

    // set speed to 200 out of possible range 0~255
    analogWrite(En, speed);
  }
  else
  {
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
  }
}

void SetupMotor()
{
  pinMode(MOT_ENABLE_1_PIN, OUTPUT);
  pinMode(MOT_ENABLE_2_PIN, OUTPUT);

  pinMode(MOT_IN_1_PIN, OUTPUT);
  pinMode(MOT_IN_2_PIN, OUTPUT);
  pinMode(MOT_IN_3_PIN, OUTPUT);
  pinMode(MOT_IN_4_PIN, OUTPUT);

  speedMT1 = MAX_SPEED;
  speedMT2 = MAX_SPEED;
}

void SetupDistanceSensor()
{
  distanceSensorLeft = 0;
  distanceSensorRight = 0;
  distanceSensorCenter = 0;
}

long GetDistanceForSensorUltraonic(int TRIGGER_PIN, int ECHO_PIN)
{
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(ECHO_PIN, INPUT);
  long duration = pulseIn(ECHO_PIN, HIGH);

  // convert the time into a distance
  return microsecondsToCentimeters(duration);
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

