// define libraries
#include <Arduino.h>
#include <wpi-32u4-lib.h>
#include <IRdecoder.h>
#include <ir_codes.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include <Rangefinder2.h>


// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

// Sets up rangefinder sensor
Rangefinder rangefinder(11, 4);
Rangefinder2 rangefinder2(3,2);
float distance = rangefinder.getDistance();
float distance2 = rangefinder2.getDistance();

// set up chassis
Chassis chassis(7.2, 1440, 12.7); //13.5 instead of 12.7

// set up LEDs
const int LED_PIN_EX1 = 12;
const int LED_PIN_EX2 = 18;  

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE, ROBOT_FIRE, ROBOT_RESCUE, ROBOT_WAIT};
ROBOT_STATE robotState = ROBOT_IDLE;

// define robot location
enum ROBOT_LOCATION {FIRE, HOSPITAL, INITIAL, PEOPLE};
ROBOT_LOCATION robot = INITIAL;

// TODO: find a better base and turn speed
float baseSpeed = 10.0;
float turnSpeed = 10.0;

// set LED function
void setLED(int pin, bool value)
{
  Serial.println("setLED()");
  digitalWrite(pin, value);
}


// idle() go to IDLE
void idle(void)
{
  chassis.idle();
  robotState = ROBOT_IDLE;
}


void distanceReading(){
  distance = rangefinder.getDistance();
  distance2 = rangefinder2.getDistance();
}

void fireReading(){
  /*
  TODO: fire sensor
  1. create fire sensing vars at the top
  2. add fire sensing to init() method
  3. update fire sense var in this function
  */
}

// standard setup
void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  Serial.begin(115200);
  pinMode(LED_PIN_EX1, OUTPUT);

  // TODO, Section IV.1 step5: Initialize the chassis (which also initializes the motors)
  chassis.init();
  chassis.setMotorPIDcoeffs(4, 0.02);

  idle();

  // Initializes the IR decoder
  decoder.init();

  // initialize rangefinder 
  rangefinder.init();
  rangefinder2.init();

  Serial.println("/setup()");
}

// handle key presses
void handleKeyPress(int16_t keyPress)
{
 // emergency stop button
  if (keyPress == ENTER_SAVE){
    Serial.println("STOP");
    idle();
    robotState = ROBOT_IDLE;
  }

   /*
  TODO: start button  ABBY 
  1. add button for starting a round-- switches back to drive
  */
if (keyPress == 1){
  Serial.println ("START");
  drive();
  robotState = ROBOT_DRIVE;
}

}

void continueUntilDoneOrBlocked(int forward, int angle){
  /*
  TODO: drive forwards or turn until you're done or blocked
  1- check sensors
  2- continue movement while checking sensors
  */
}

void drive(){
  //LAURA
  switch (robotState)
  {
  case FIRE:
    fireToPeople();
    break;
  case HOSPITAL:
    hospitalToFire();
    break;
  case INITIAL:
    startToFire();
    break;
  case PEOPLE:
    peopleToHospital();
    break;
  default:
    break;
  }
}

void hospitalToFire(){
  /*
  TODO: write this function to drive from hospital back to fire
  1- write function, using ultrasonic sensor and knowledge of walls/turns
  1b- maybe add PID for straight wall follow? Optional and may not work
  2- update location
  3- update state


  Note: our fire location is the top one, use the route that goes next to the wall furtherst from
  fires, then crosses through the middle towards fire (just trying to avoid other robot)
  */
}

void startToFire(){
/*
  SAM
  TODO: write this function to drive from start location to fire
  1- write function, using ultrasonic sensor and knowledge of walls/turns
  1b- maybe add PID for straight wall follow? Optional and may not work
  2- update location
  3- update state

  Note: our start location is further into the fire department, as in the other robot is going first
  fire location is the top one, use the route that goes next to the wall furtherst from
  fires, then crosses through the middle towards fire (just trying to avoid other robot)
  */
}

void fireToPeople(){
  /*
  TODO: write this function to drive from fire to the people
  1- write function, using ultrasonic sensor and knowledge of walls/turns
  1b- maybe add PID for straight wall follow? Optional and may not work
  2- update location
  3- update state

  Note: our fire location is the top one
  */
}

void peopleToHospital(){
/*
  TODO: write this function to drive from people to the hospital
  1- write function, using ultrasonic sensor and knowledge of walls/turns
  1b- maybe add PID for straight wall follow? Optional and may not work
  2- update location

  Note: our fire location is the top one, use the route that goes next to the wall furtherst from
  fires, then crosses through the middle towards fire (just trying to avoid other robot)
  */
}

void drive(){
  /*
  TODO: write drive method
  1- create a switch case for each location start point (FIRE, HOSPITAL, INITIAL, PEOPLE)
  2- call relevant drive function (hospitalToFire, startToFire, fireToPeople, peopleToHospital)
  */

  switch (robotState){
    case FIRE:
      fireToPeople();
      break;
    
    case HOSPITAL:
      hospitalToFire();
      break;
    
    case INITIAL:
      startToFire();
      break;
    
    case PEOPLE:
      peopleToHospital();
      break;
    
    default:
      break;
  } 

}
void rescue(){
  /*
  TODO: rescue people
  1- scoop up people
  2- lift and drop can to check we've grabbed it
  3- IF NOT, adjust and retry
  4- ONCE GRABBED, switch to drive 
  */
}

void fire(){
  /*
  TODO: put out fire
  1- adjust so facing fire
  2- blow flame out
  3- check flame is out with sensor
  4- switch to drive and continue
  */
}

void wait(int time){
  /*
  Wait function while other robot goes wherever
  */
  delay(time);
  robotState = ROBOT_DRIVE;
}

// main loop
void loop()
{
  // Checks for a key press on the remote
  int16_t keyPress = decoder.getKeyCode();
  if(keyPress >= 0) handleKeyPress(keyPress);

  /*
  1- go through each state, idle does nothing, drive moves between locations,
  fire puts out fire, rescue scoops up people. this needs to call relevant functions.
  (idle, drive, rescue, fire)
  2- add in corresponding LED light choices for each state (choose whatever you feel like)
  */
  switch(robotState)
  {
    case ROBOT_DRIVE:
      setLED(LED_PIN_EX1, HIGH);
      setLED(LED_PIN_EX2, HIGH);
      drive();
      break;
    
    case ROBOT_FIRE:
      setLED(LED_PIN_EX1, HIGH);
      setLED(LED_PIN_EX2, LOW);
      fire();
      break;

    case ROBOT_RESCUE:
      setLED(LED_PIN_EX1, LOW);
      setLED(LED_PIN_EX2, HIGH);
      rescue();
      break;

    case ROBOT_WAIT:
      setLED(LED_PIN_EX1, LOW);
      setLED(LED_PIN_EX2, LOW);
      wait(10);
      break;
    
    case ROBOT_IDLE:
      setLED(LED_PIN_EX1, LOW);
      setLED(LED_PIN_EX2, LOW);
      break;

    default:
      break;
  }
}


