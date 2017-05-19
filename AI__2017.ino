#include "SharpIR.h"
#include "BluetoothDevice.h"
#include <SoftwareSerial.h>
//Distance Sensor Left
#define DS_TRIGGER_1_PIN   4
#define DS_ECHO_1_PIN   5

//Distance Sensor Right
#define DS_TRIGGER_2_PIN  12
#define DS_ECHO_2_PIN     13

//Distance Sensor Sharp IR Left
#define   IR_PIN_LEFT     A0

//Distance Sensor Sharp IR  Right
#define   IR_PIN_RIGHT    A1
#define   MODEL_IR        1080


//Distance Sensor Center
#define DS_TRIGGER_3_PIN  A2
#define DS_ECHO_3_PIN     A3


//Motor LEFT
#define MOT_ENABLE_1_PIN  5
#define MOT_IN_1_PIN    7
#define MOT_IN_2_PIN    8


//Motor RIGHT
#define MOT_ENABLE_2_PIN  6
#define MOT_IN_3_PIN    10
#define MOT_IN_4_PIN    11


//Speed Sensor
#define SS_D0_PIN       2
#define SS_D1_PIN       3

//Speed limit
#define MAX_SPEED         200
#define MID_SPEED         150
#define LOW_SPEED         110
#define MIN_SPEED         80

//Collision Distance
#define EX_LOW_DISTANCE   10
#define LOWER_DISTANCE      18
#define LOW_DISTANCE    20
#define MED_DISTANCE    60
#define HIGH_DISTANCE     100

//Direction
#define DIR_12        12
#define DIR_2       22
#define DIR_4       44
#define DIR_6       66
#define DIR_8       88
#define DIR_10        10
#define STOP        -1

//Change Direction Step
#define TURN_COLLISION    1
#define TURN_STEP         3
#define MOVE_BACK_STEP    8

//Loop step
#define TIME_STEP     50

//LED
#define LED_1_PIN     13
#define LED_2_PIN     12
#define LED_3_PIN     A5

//Macro
#define HAVE_COLLISION(x, d)  (x < d)

//Balance
#define BALANCE_RATIO 0.15
#define DISTANCE_MAX 500
int distanceSensorLeft, distanceSensorRight, distanceSensorCenter, speedSensorLeft, speedSensorRight;

//// BLUETOOTH DEVICE.
#define   BLT_RX_PIN                13
#define   BLT_TX_PIN                12
int speedMT1, speedMT2;
int timer, direction, nextDirection, changeDirectionStep;
int turn, delaystep, countdir = 0;
BluetoothDevice                     blueDevice;
SharpIR irLeft(IR_PIN_LEFT, MODEL_IR);
SharpIR irRight(IR_PIN_RIGHT, MODEL_IR);
bool isturnback = false;
bool isfollowright;
bool isfollowall;

int oldturn = 0;

// put your setup code here, to run once:
void setup()
{
  // initialize serial communication:
  Serial.begin(9600);

  //Setup distance sensor
  SetupDistanceSensor();

  //Setup motor
  SetupMotor();

  //Setup speed sensor
  //SetupSpeedSensor();

  //Setup debug LED
  SetupLED();

  // Setup Debug With Bluetooth.
  blueDevice.SetPinBLT(BLT_RX_PIN, BLT_TX_PIN);

  //setup timer
  timer = 0;

  //setup defaul direction
  direction = DIR_12;
  changeDirectionStep = 0;
  turn = TURN_STEP;

  //anti stuck
  delaystep = 0;
  countdir = 0;
  isturnback = false;
  isfollowright = true;
  isfollowall = true;
  oldturn = 0;
}

void loop()
{
  // put your main code here, to run repeatedly:
  distanceSensorLeft = irLeft.distance();
  distanceSensorRight = irRight.distance();
  distanceSensorCenter = GetDistanceForSensorUltraonic(DS_TRIGGER_3_PIN, DS_ECHO_3_PIN);

  if (distanceSensorLeft == 0)
    distanceSensorLeft = DISTANCE_MAX;

  if (distanceSensorRight == 0)
    distanceSensorRight = DISTANCE_MAX;

  if (distanceSensorCenter == 0)
    distanceSensorCenter = DISTANCE_MAX;

  if (timer % 3 == 0)
  {
    Serial.print("ChangeDirectionStep: ");
    Serial.print(changeDirectionStep);
    Serial.print(" - Dir: ");
    Serial.print(direction);
    Serial.print(" - DS1: ");
    Serial.print(distanceSensorLeft);
    Serial.print(" - DS2: ");
    Serial.print(distanceSensorRight);
    Serial.print(" - DS3: ");
    Serial.print(distanceSensorCenter);
    Serial.println();
  }

  switch (direction)
  {
    case DIR_12:
      countdir ++;

      if (HAVE_COLLISION(distanceSensorCenter, LOW_DISTANCE))
      {
        speedMT1 = MIN_SPEED;
        speedMT2 = MIN_SPEED;

        // chi cho bam tuong
        if (isturnback && isfollowall)
        {
          if (isfollowright)
            direction = DIR_10;
          else
            direction = DIR_2;
          turn = TURN_STEP + (isturnback ? TURN_STEP : 0);
          isturnback = false;
        }
        else
        {
          if (distanceSensorLeft > distanceSensorRight)
          {
            direction = DIR_10;
            turn = TURN_STEP + (isturnback ? TURN_STEP : (distanceSensorRight < LOWER_DISTANCE) ? TURN_COLLISION : 0);
            isturnback = false;
          }
          else
          {
            direction = DIR_2;
            turn = TURN_STEP + (isturnback ? TURN_STEP : (distanceSensorLeft < LOWER_DISTANCE) ? TURN_COLLISION : 0);
            isturnback = false;
          }
        }
      }
      else
      {
        if (HAVE_COLLISION(distanceSensorCenter, MED_DISTANCE))
        {
          if (speedMT1 > LOW_SPEED || speedMT2 > LOW_SPEED)
          {
            speedMT1 = (1 - 0.2) * speedMT1;
            speedMT2 = (1 - 0.2) * speedMT2;
          }
        }
        else // > MED_DISTANCE
        {
          speedMT1 = MAX_SPEED;
          speedMT2 = MAX_SPEED;
        }

        if (isfollowall)
        {
          int sensorCheck = isfollowright ? distanceSensorRight : distanceSensorLeft;

          if (sensorCheck > 32)
          {
            direction = isfollowright ? DIR_2 : DIR_10;
            turn = TURN_COLLISION + ((sensorCheck > 60) ? TURN_STEP : 0);
            if (oldturn + turn > 5)
              turn = 5 - oldturn;
            isturnback = false;
          }

          if (sensorCheck < 12)
          {
            direction = isfollowright ? DIR_10 : DIR_2;
            turn = TURN_COLLISION;
            isturnback = false;
          }
        }
        else
        {
          if (HAVE_COLLISION(distanceSensorLeft, EX_LOW_DISTANCE) )
          {
            direction = DIR_2;
            turn = TURN_COLLISION + (isturnback ? TURN_STEP : 0);
            isturnback = false;
          }

          if (HAVE_COLLISION(distanceSensorRight, EX_LOW_DISTANCE) )
          {
            direction = DIR_10;
            turn = TURN_COLLISION + (isturnback ? TURN_STEP : 0);
            isturnback = false;
          }
        }
      }

      break;

    case DIR_6:
      speedMT1 = MIN_SPEED;
      speedMT2 = MIN_SPEED;

      if ((!HAVE_COLLISION(distanceSensorCenter, LOW_DISTANCE)) && changeDirectionStep == 0)
      {
        isturnback = true;
        changeDirectionStep = 0;
        direction = STOP;
        nextDirection = DIR_12;
        countdir = 0;
        break;
      }

      changeDirectionStep++;

      if (changeDirectionStep >= turn)
      {
        isturnback = true;
        Serial.println();
        Serial.print("___________________ DIR_6");
        Serial.println();

        changeDirectionStep = 0;
        direction = STOP;
        nextDirection = DIR_12;
      }

      countdir = 0;
      break;

    case DIR_2:
      speedMT1 = MID_SPEED;
      speedMT2 = MID_SPEED;

      changeDirectionStep++;
      if (changeDirectionStep >= turn || (!HAVE_COLLISION(distanceSensorCenter, MED_DISTANCE)))
      {
        Serial.println();
        Serial.print("___________________ DIR_2");
        Serial.println();
        oldturn = changeDirectionStep;
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
      speedMT1 = MID_SPEED;
      speedMT2 = MID_SPEED;

      changeDirectionStep++;
      if (changeDirectionStep >= turn || (!HAVE_COLLISION(distanceSensorCenter, MED_DISTANCE)))
      {
        Serial.println();
        Serial.print("___________________ DIR_10");
        Serial.println();
        oldturn = changeDirectionStep;
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
      changeDirectionStep++;
      if (changeDirectionStep >= 1)
      {
        Serial.println();
        Serial.print("___________________ STOP");
        Serial.println();

        changeDirectionStep = 0;
        direction = nextDirection;
      }
      break;
  }

  if (isfollowall)
    speedBalance(speedMT1, speedMT2, distanceSensorLeft, distanceSensorRight, isfollowright ? 19 : -19);
  else
    speedBalance(speedMT1, speedMT2, distanceSensorLeft, distanceSensorRight, 0);

  RunCar(direction, speedMT1, speedMT2);


  Serial.print("isturnback: ");
  Serial.print(isturnback);

  String temp = "ML:";
  temp += speedMT1;
  temp += " :";
  temp += "MR:";
  temp += speedMT2;
  temp += ": ";
  temp += "SSL:";
  temp += distanceSensor1;
  temp += ": ";
  temp += "SSR:";
  temp += distanceSensor2;
  temp += ": ";
  temp += "SSH:";
  temp += distanceSensor3;
  temp += ": ";
  temp += direction;
  blueDevice.SentToBluetoothDevice(temp);

  timer++;
  if (timer == 10000) timer = 0;
  delay(TIME_STEP);
}

void SetupLED()
{
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  pinMode(LED_3_PIN, OUTPUT);
}

void TurnLED(int pin, int mode)
{
  if (mode == 1)
  {
    digitalWrite(pin, HIGH);  // turn the LED on (HIGH is the voltage level)
  }
  else
  {
    digitalWrite(pin, LOW);   // turn the LED off by making the voltage LOW
  }
}

void RunCar(int dir, int speedM1, int speedM2)
{
  if (dir == DIR_12)
  {
    RunMotor(1, speedM1, MOT_ENABLE_1_PIN, MOT_IN_1_PIN, MOT_IN_2_PIN);
    RunMotor(1, speedM2, MOT_ENABLE_2_PIN, MOT_IN_3_PIN, MOT_IN_4_PIN);
  }
  if (dir == DIR_2)
  {
    RunMotor(-1, speedM1, MOT_ENABLE_1_PIN, MOT_IN_1_PIN, MOT_IN_2_PIN);
    RunMotor(1, speedM2, MOT_ENABLE_2_PIN, MOT_IN_3_PIN, MOT_IN_4_PIN);
  }
  if (dir == DIR_10)
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
    RunMotor(0, speedM1, MOT_ENABLE_1_PIN, MOT_IN_1_PIN, MOT_IN_2_PIN);
    RunMotor(0, speedM2, MOT_ENABLE_2_PIN, MOT_IN_3_PIN, MOT_IN_4_PIN);
  }
  if (dir == STOP)
  {
    RunMotor(-1, speedM1, MOT_ENABLE_1_PIN, MOT_IN_1_PIN, MOT_IN_2_PIN);
    RunMotor(-1, speedM2, MOT_ENABLE_2_PIN, MOT_IN_3_PIN, MOT_IN_4_PIN);
  }
}

void speedBalance(int &speed1, int &speed2, long distance1, long distance2, long delta)
{
  float ratio = BALANCE_RATIO;

  if (distance1 > distance2 + delta)
  {
    if (speed1 * (1 + ratio) <= 255)
      speed1 = speed1 * (1 + ratio);
    else
      speed2 = speed2 * (1 - ratio);
  }
  else if (distance1 < distance2 + delta)
  {
    if (speed2 * (1 + ratio) <= 255)
      speed2 = speed2 * (1 + ratio);
    else
      speed1 = speed1 * (1 - ratio);
  }
}

//void SetupSpeedSensor()
//{
//  pinMode(SS_D0_PIN, INPUT_PULLUP);
//  pinMode(SS_D1_PIN, INPUT_PULLUP);
//
//  attachInterrupt(digitalPinToInterrupt(SS_D0_PIN), SpeedSensor1, RISING );
//  attachInterrupt(digitalPinToInterrupt(SS_D1_PIN), SpeedSensor2, RISING );
//
//  speedSensor1 = 0;
//  speedSensor2 = 0;
//}

//void SpeedSensor1()
//{
//  speedSensor1++;
//}
//
//void SpeedSensor2()
//{
//  speedSensor2++;
//}

void RunMotor(int dir, int speed, int En, int In1, int In2)
{
  if (dir == 0)
  {
    // turn on motor A
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);

    // set speed to 200 out of possible range 0~255
    analogWrite(En, speed);
  }
  else if (dir == 1)
  {
    // turn on motor A
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);

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
