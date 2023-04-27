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

// Set up servo motor
Servo32U4 servo;
int SERVO_DOWN = 1000;
int SERVO_UP = 2000;

// set up LEDs
const int LED_PIN_EX1 = 12;
const int LED_PIN_EX2 = 18;  

// connect flame sensor to pin A11
const int FLAME_PIN = A11; 
int flameSignal=analogRead(FLAME_PIN);

// fan pin and speed
const int FAN_PIN = 20; 
const int FAN_SPEED = 255; 

// Defines the robot states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE, ROBOT_FIRE, ROBOT_RESCUE, ROBOT_WAIT, ROBOT_FLEE};
ROBOT_STATE robotState = ROBOT_IDLE;

// Define robot location
enum ROBOT_LOCATION {FIRE, HOSPITAL, INITIAL, PEOPLE, GATE};
ROBOT_LOCATION robotLocation = INITIAL;

// Base and turn speed
float baseSpeed = 20.0;
float turnSpeed = 100.0;

/**
 * Function to set LED
*/
void setLED(int pin, bool value)
{
  digitalWrite(pin, value);
}

/**
 * Switches the robot state to idle
*/
void idle(void)
{
  chassis.idle();
  robotState = ROBOT_IDLE;
}

/**
 * Add white line sensing method so we can check if we've entered an area
*/

void distanceReading(){
  distance = rangefinder.getDistance();
  delay(100);
  distance2 = rangefinder2.getDistance();
  delay(100);
}

/**
 * Activates the fire sensor 
*/

void fireReading(){
  flameSignal=analogRead(FLAME_PIN);
}

/**
 * Standard setup method
*/

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

  // Initialize rangefinder 
  rangefinder.init();
  rangefinder2.init();

  // Initializes servo motor
  servo.attach();
  servo.setMinMaxMicroseconds(SERVO_DOWN, SERVO_UP);
  Serial.println("/setup()");
}

/**
 *  Method that turns the robot the amount of the given angle
*/
void turn(int angle){
  chassis.turnFor(-angle, turnSpeed, true);
}

/**
 * Drives forward until certain distance away from wall
 * then turns until correct wall distance is reached again
*/
void continueUntilDone(int distanceToWall, int angle){
  
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
/**
 * Checks if there is a fire sensed
 * If a fire is sensed return true.
*/
bool checkForFire(){
  
  fireReading();
  if (flameSignal < 350){
    return true;
  } else {
    return false;
  }
}

/**
 * Directs the robot from the hospital to the fire
*/

void hospitalToFire(){

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

 // into fire
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

/**
 * Directs the robot from the fire department to the fire
*/

void startToFire(){

  Serial.println("Start!");

 Serial.println("switching to fire");

 robotLocation = FIRE;

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

/**
 * Directs the robot from the fire to the people in need of rescue
*/

void fireToPeople(){

 // straight a bit, then right
  continueUntilDone(57, 100);

 // pickup time!
 robotLocation = PEOPLE;
 robotState = ROBOT_RESCUE;

}

/**
 * Directs the robot from the rescue zone to the hospital
*/

void peopleToHospital(){

  // Route from the rescue zone to the hospital
  turn(200);
  continueUntilDone(25, -95);
  continueUntilDone(25, -80);
  continueUntilDone(30, 110);
  continueUntilDone(60, 110);

  // update location
  robotLocation = HOSPITAL;
  robotState = ROBOT_IDLE;
}

/**
 * When starting at the gate, directs the robot from the gate to the fire
*/

void gateToFire(){

 // Route from gate to fire
  continueUntilDone(83.2, 115);
  continueUntilDone(30, -105);
  robotLocation = FIRE;

  // Once location is reached, check for fire
  if (checkForFire()){
  robotState = ROBOT_FIRE;
  }
  else{
  robotState = ROBOT_FLEE;
  }
 
}

/**
 * If there is no fire found, go out of the gate 
 * and set robot state back to idle
*/

void fireToGate(){
 
 // Route from fire to gate
  turn(-100);
  continueUntilDone(25, -100);
  continueUntilDone(30, 210);
  robotLocation = GATE;
  robotState = ROBOT_IDLE;
}


 /**
  * Directs the robot from the first fire to the second fire
 */

void fire1ToFire2(){

  // Route from fire 1 to fire 2
  turn(-100);
  continueUntilDone(25, -100);
  continueUntilDone(20, -80);
  continueUntilDone(150, 110);
  continueUntilDone(30, 100);
  continueUntilDone(60, 100);
  continueUntilDone(30, -100);
}

/**
 * Calls the methods to direct the robot from place to place
*/

void drive(){
  
  switch (robotLocation)
  {
    // If at the fire , go rescue the people
  case FIRE:
    Serial.println("fire to people time...");
    fireToPeople();
    break;

    // If at the hospital, go to the fire
  case HOSPITAL:
    hospitalToFire();
    break;

    // If at the starting location, go to the fire
  case INITIAL:
    startToFire();
    Serial.print("robot state? ");
    break;

    // If at the rescue zone, go to the hospital 
  case PEOPLE:
    peopleToHospital();
    break;

    // If at the gate, drive to the fire
  case GATE:
    gateToFire();
    break;
  default:
    break;
  }
}

/**
 * Allows the robots actions to relate to the buttons pressed on the remote
*/

void handleKeyPress(int16_t keyPress)
{

  Serial.println(keyPress);

 // emergency stop button
  if (keyPress == ENTER_SAVE){
    Serial.println("STOP");
    idle();
    robotState = ROBOT_IDLE;
  }

   
 // Button for starting around, switches back to drive

  if (keyPress == 16){ // key code for 1
    Serial.println ("START");
    // Serial.println("switching to drive");
    robotState = ROBOT_DRIVE;
    
  }

}

/**
 *  Method that activates the robots arm in order to rescue the people
*/
void rescue(){
  /* LAURA
  TODO: rescue people
  1- scoop up people
  2- lift and drop can to check we've grabbed it
  3- IF NOT, adjust and retry
  4- ONCE GRABBED, switch to drive 
  */
  

}

 /*
  Wait function while other robot goes wherever
  */

void wait(int time){
 
  delay(time);
}

/**
 * Method to activate fan if a fire is sensed
*/

void fire(){
  Serial.println((String)"flame sensor reading:" + flameSignal);
 if(checkForFire()){
  analogWrite(FAN_PIN,FAN_SPEED);
  Serial.println("Fan is turned on");
  analogWrite(FAN_PIN,0); //turn off fan 
  Serial.println("Fan is turned off");
  robotState=ROBOT_RESCUE; //then rescue
 } else {
  robotState=ROBOT_FLEE;
 }

}


/**
 * Method to loop through all the different robot states
*/
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
    // Sets the robot into motion
    case ROBOT_DRIVE:
      setLED(LED_PIN_EX1, HIGH);
      setLED(LED_PIN_EX2, HIGH);
      drive();
      break;
    
    // Activates the methods to put out the fire
    case ROBOT_FIRE:
      Serial.println("FIRE!");
      setLED(LED_PIN_EX1, HIGH);
      setLED(LED_PIN_EX2, LOW);
      fire();
      break;

    // Activates the methods to save the people
    case ROBOT_RESCUE:
      setLED(LED_PIN_EX1, LOW);
      setLED(LED_PIN_EX2, HIGH);
      rescue();
      break;

    // Waits for the other robot
    case ROBOT_WAIT:
      setLED(LED_PIN_EX1, LOW);
      setLED(LED_PIN_EX2, LOW);
      wait(10);
      break;
    
    // Stops the robot and puts it into idle
    case ROBOT_IDLE:
      setLED(LED_PIN_EX1, LOW);
      setLED(LED_PIN_EX2, LOW);
      break;

    // Calls the method to send the robot outside the gate  
    case ROBOT_FLEE:
      setLED(LED_PIN_EX1, HIGH);
      setLED(LED_PIN_EX2, HIGH);
      fireToGate();
      break;

    default:
      break;
  }
}


