// define libraries
#include <Arduino.h>
#include <wpi-32u4-lib.h>
#include <IRdecoder.h>
#include <ir_codes.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include <Rangefinder2.h>
#include <servo32u4.h>

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

//Set up servo motor
Servo32U4 servo;
int SERVO_DOWN = 1000;
int SERVO_UP = 2500;

// set up LEDs
const int LED_PIN_EX1 = 12;
const int LED_PIN_EX2 = 18;  

// connect flame sensor to pin A11
const int FLAME_PIN = A11; 
int flameSignal=analogRead(FLAME_PIN);

//fan pin and speed
const int FAN_PIN = 20; 
const int FAN_SPEED = 255; 

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE, ROBOT_FIRE, ROBOT_RESCUE, ROBOT_WAIT, ROBOT_FLEE};
ROBOT_STATE robotState = ROBOT_RESCUE;

// define robot location
enum ROBOT_LOCATION {FIRE, HOSPITAL, INITIAL, PEOPLE, GATE};
ROBOT_LOCATION robotLocation = FIRE;

// TODO: find a better base and turn speed
float baseSpeed = 20.0;
float turnSpeed = 100.0;


// set LED function
void setLED(int pin, bool value)
{
  // Serial.println("setLED()");
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
  delay(100);
  distance2 = rangefinder2.getDistance();
  delay(100);
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
  pinMode(FAN_PIN,OUTPUT);

  chassis.init();
  chassis.setMotorPIDcoeffs(4, 0.02);

  idle();

  // Initializes the IR decoder
  decoder.init();

  // initialize rangefinder 
  rangefinder.init();
  rangefinder2.init();

  // initializes servo motor
  servo.attach();
  servo.setMinMaxMicroseconds(SERVO_DOWN, SERVO_UP);
  Serial.println("/setup()");
}

void turn(int angle){
  chassis.turnFor(-angle, turnSpeed, true);
}

void continueUntilDone(int distanceToWall, int angle){
  /*
  drive forwards or turn until you're done, then turn until done
  1- check sensors
  2- continue movement while checking sensors

  TODO: potentially add PID, but don't have to, eg use wall following to not get off track
  */

 // RIGHT: positive angle!

 distanceReading();

 while (distance > distanceToWall) {
  distanceReading();
  chassis.setWheelSpeeds(baseSpeed,baseSpeed);
 }
 chassis.turnFor(-angle, turnSpeed, true);
 Serial.println("wait!");
 delay(5);
  distanceReading();
}

bool checkForFire(){
  // TODO: check if there's fire, if there is return true - OLIVIA
  fireReading();
  if (flameSignal < 100){
    return true;
  } else {
    return false;
  }
}

void hospitalToFire(){
  /*
  goes from hospital to fire

  Note: our fire location is the top one, use the route that goes next to the wall furtherst from
  fires, then crosses through the middle towards fire (just trying to avoid other robot)
  
  TODO: test values!

  */

 // turn away from hospital
  turn(110);
 // go forwards, turn left
 continueUntilDone(30, -95);
  // forwards (center line), turn left
 continueUntilDone(126, -85);
 // long forwards, turn right
 continueUntilDone(85, 35);
  continueUntilDone(35, 105);
 // go forwards, turn right
 continueUntilDone(83.2, 115);

 //into fire
 continueUntilDone(30, -105);
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
  Serial.println("Start!");
/*
  Note: our start location is further into the fire department, as in the other robot is going first
  fire location is the top one, use the route that goes next to the wall furtherst from
  fires, then crosses through the middle towards fire (just trying to avoid other robot)
  
  TODO: test these distances!!!!!
  */

 Serial.println("switching to fire");
 robotLocation = FIRE;

 // go forwards, turn right
 //Serial.println("forward right!");
 //continueUntilDone(15, 100);
 // go forwards, turn left
 Serial.println("forward left!");
 Serial.println("new distance!");
 distanceReading();
 continueUntilDone(25, -90);
 // forwards, turn left
 Serial.println("forward left!");
 continueUntilDone(20, -90);
 // forwards, turn right
 Serial.println("forward right!");
 continueUntilDone(130, 120);
 // forwards til close
 Serial.println("forward right!");
 continueUntilDone(35, 105);
 // forwards, right
 Serial.println("forward right!");
 continueUntilDone(85, 115);
 Serial.println("into the fire!");
 continueUntilDone(30, -105);

  if (checkForFire()){
  robotState = ROBOT_FIRE;
 }
 else{
  robotState = ROBOT_FLEE;
 }

}

void fireToPeople(){

  /*
  go from fire to people

  Note: our fire location is the top one

  TODO: test values!
  */

  // straight a bit, then right
  continueUntilDone(57, 100);
 // pickup time!
 //robotLocation = PEOPLE;
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
  Serial.println("Start!");
  Serial.println("turn 180");
  turn(200);
  Serial.println("go straight and turn left");
  continueUntilDone(25, -95);
  Serial.println("go straight and turn left");
  continueUntilDone(25, -80);
  Serial.println("go straight and turn right");
  continueUntilDone(30, 110);
  Serial.println("go straight and turn right");
  continueUntilDone(65, 110);
  Serial.println("go straight and turn left");
  continueUntilDone(60, 110);

  //update location
  robotLocation = HOSPITAL;
  robotState = ROBOT_IDLE;
}

void gateToFire(){
  // TODO: if starting at gate, go from here to fire
  continueUntilDone(83.2, 115);
  //into fire
  continueUntilDone(30, -105);
  robotLocation = FIRE;
  if (checkForFire()){
  robotState = ROBOT_FIRE;
  }
  else{
  robotState = ROBOT_FLEE;
  }
 
}

void fireToGate(){
  // if no fire, go out of gate, go back to idle
  turn(-100);
  continueUntilDone(25, -100);
  continueUntilDone(30, 210);
  robotLocation = GATE;
  robotState = ROBOT_IDLE;
}

void fire1ToFire2(){
  turn(-100);
  continueUntilDone(25, -100);
  continueUntilDone(20, -80);
  continueUntilDone(150, 110);
  continueUntilDone(30, 100);
  continueUntilDone(60, 100);
  continueUntilDone(30, -100);
}


void drive(){
  //LAURA
  switch (robotLocation)
  {
  case FIRE:
    Serial.println("fire tp people time...");
    fireToPeople();
    break;
  case HOSPITAL:
    hospitalToFire();
    break;
  case INITIAL:
    startToFire();
    Serial.print("robot state? ");
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
  Serial.println(keyPress);
 // emergency stop button
  if (keyPress == ENTER_SAVE){
    Serial.println("STOP");
    // TODO: figure out how to make this work while also in drive mode
    idle();
    robotState = ROBOT_IDLE;
  }

   /*
  button for starting a round-- switches back to drive
  */

if (keyPress == 16){ // key code for 1
  Serial.println ("START");
  // Serial.println("switching to drive");
  robotState = ROBOT_DRIVE;
}

}

void rescue(){
  /* LAURA
  TODO: rescue people
  1- scoop up people
  2- lift and drop can to check we've grabbed it
  3- IF NOT, adjust and retry (check with rangefinder sensor?)
  4- ONCE GRABBED, switch to drive 
  */
  distanceReading();
  Serial.println(distance);
  //arm down and go straight 
  continueUntilDone()
  while (distance < 4) {
    
    Serial.println(distance);
    servo.writeMicroseconds(SERVO_UP);
    delay(2000);
    servo.writeMicroseconds(SERVO_DOWN);
    delay(2000);
    distanceReading();
  }
  distanceReading();
  //robotState = ROBOT_DRIVE;
  idle();
  Serial.println("people rescued!");
}

void fire(){
 if(checkForFire()){
  analogWrite(FAN_PIN,FAN_SPEED);
  Serial.println("Fan is turned on");
 } else{
  robotState=ROBOT_FLEE;
 }
}

void wait(int time){
  /*
  Wait function while other robot goes wherever
  */
  delay(time);
  Serial.println("switching to drive");
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
      Serial.println("FIRE!");
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


