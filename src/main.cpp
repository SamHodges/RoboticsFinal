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

// connect flame sensor to pin A11
const int FLAME_PIN = A11; 
int flameSignal=analogRead(FLAME_PIN);

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE, ROBOT_FIRE, ROBOT_RESCUE, ROBOT_WAIT, ROBOT_FLEE};
ROBOT_STATE robotState = ROBOT_IDLE;

// define robot location
enum ROBOT_LOCATION {FIRE, HOSPITAL, INITIAL, PEOPLE, GATE};
ROBOT_LOCATION robotLocation = INITIAL;

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

// TODO: add white line sensing method so we can check if we've entered an area

void distanceReading(){
  distance = rangefinder.getDistance();
  distance2 = rangefinder2.getDistance();
}

void fireReading(){
  flameSignal=analogRead(FLAME_PIN);
}

// standard setup
void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  Serial.begin(115200);
  pinMode(LED_PIN_EX1, OUTPUT);
  pinMode(FLAME_PIN,INPUT);

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

void continueUntilDone(int distanceToWall, int angle){
  /*
  drive forwards or turn until you're done, then turn until done
  1- check sensors
  2- continue movement while checking sensors

  TODO: potentially add PID, but don't have to, eg use wall following to not get off track
  */

 // RIGHT: positive angle!

 while (distance > distanceToWall) {
  distanceReading();
  chassis.setWheelSpeeds(baseSpeed,baseSpeed);
 }
 chassis.turnFor(-angle, turnSpeed, true);
}

bool checkForFire(){
  // TODO: check if there's fire, if there is return true
}

void hospitalToFire(){
  /*
  goes from hospital to fire

  Note: our fire location is the top one, use the route that goes next to the wall furtherst from
  fires, then crosses through the middle towards fire (just trying to avoid other robot)
  
  TODO: test values!
  */

 // turn away from hospital
  continueUntilDone(0, 180);
 // go forwards, turn left
 continueUntilDone(10, -90);
 // forwards (center line), turn left
 continueUntilDone(50, -90);
 // long forwards, turn right
  continueUntilDone(10, 90);
 // go forwards, turn right
 continueUntilDone(20, 90);
 // switch to fire!!
 robotLocation = FIRE;
 if (checkForFire()){
  robotState = ROBOT_FIRE;
 }
 else{
  robotState = ROBOT_FLEE;
 }
}

void startToFire(){
/*
  Note: our start location is further into the fire department, as in the other robot is going first
  fire location is the top one, use the route that goes next to the wall furtherst from
  fires, then crosses through the middle towards fire (just trying to avoid other robot)
  
  TODO: test these distances!!!!!
  */

 // go forwards, turn right
  continueUntilDone(5, 90);
 // go forwards, turn left
 continueUntilDone(10, -90);
 // forwards, turn left
 continueUntilDone(10, -90);
 // forwards, turn right
 continueUntilDone(50, 90);
 // forwards til close
 continueUntilDone(10, 90);
 // forwards, right
 continueUntilDone(20, 90);
 // switch to fire!
 robotLocation = FIRE;
 robotState = ROBOT_FIRE;


}

void fireToPeople(){

  /*
  go from fire to people

  Note: our fire location is the top one

  TODO: test values!
  */

 // turn left
  continueUntilDone(0, -90);
 // straight a bit, then right
  continueUntilDone(15, 90);
 // pickup time!
 robotLocation = PEOPLE;
 robotState = ROBOT_RESCUE;

}

void peopleToHospital(){
/* LAURA
  TODO: write this function to drive from people to the hospital
  1- write function, using ultrasonic sensor and knowledge of walls/turns
  1b- maybe add PID for straight wall follow? Optional and may not work
  2- update location

  Note: our fire location is the top one, use the route that goes next to the wall furtherst from
  fires, then crosses through the middle towards fire (just trying to avoid other robot)
  */

  //turn left 180 degrees and go straigth 
  continueUntilDone(10, -180);
  //turn left 90 degrees and go straigth 
  continueUntilDone(10, -90);
  //turn left 90 degrees and go straigth 
  continueUntilDone(10, -90);
  //turn right 90 degrees and go straight 
  continueUntilDone(10, 90);
  //turn right 90 degrees and go straight 
  continueUntilDone(10, 90);

  //update location
  robotLocation = HOSPITAL;
}

void gateToFire(){
  // TODO: if starting at gate, go from here to fire
}

void fireToGate(){
  // if no fire, go out of gate, go back to idle
}


void drive(){
  //LAURA
  switch (robotLocation)
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
  case GATE:
    gateToFire();
    break;
  default:
    break;
  }
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
  button for starting a round-- switches back to drive
  */
if (keyPress == 1){
  Serial.println ("START");
  drive();
  robotState = ROBOT_DRIVE;
}

}

void rescue(){
  /* LAURA
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

    case ROBOT_FLEE:
      setLED(LED_PIN_EX1, HIGH);
      setLED(LED_PIN_EX2, HIGH);
      fireToGate();
      break;

    default:
      break;
  }
}


