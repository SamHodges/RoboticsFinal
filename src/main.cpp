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
int SERVO_DOWN = 4000;
int SERVO_UP = 4000;

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
ROBOT_STATE robotState = ROBOT_FIRE;

// Define robot location
enum ROBOT_LOCATION {FIRE, HOSPITAL, INITIAL, PEOPLE, GATE};
ROBOT_LOCATION robotLocation = FIRE;

// Base and turn speed
float baseSpeed = 20.0;
float turnSpeed = 100.0;

// keep track of which robot it is
int robotNumber = 1;

// line sensing
int leftLineSensorReading = 0;
int rightLineSensorReading = 0;


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

  // initializes line sensor
  pinMode(LEFT_LINE_SENSE, INPUT);
  pinMode(RIGHT_LINE_SENSE, INPUT);
}

/*
  Read line sensing
*/
void lineSensing(){
  leftLineSensorReading = analogRead(LEFT_LINE_SENSE);
  rightLineSensorReading = analogRead(RIGHT_LINE_SENSE);
}

void handleLineFollowing(float baseSpeed)
{
  lineSensing();

  int16_t error = leftLineSensorReading - rightLineSensorReading;
  float turnEffort = error * 0.17; //edit k_p!
  chassis.setTwist(baseSpeed, turnEffort);


}

bool checkIntersectionEvent()
{
  static bool prevIntersection = false;
  int16_t whiteThreshold = 50;

  bool retVal = false;


  int16_t leftADC = analogRead(LEFT_LINE_SENSE);
  int16_t rightADC = analogRead(RIGHT_LINE_SENSE);


  // TODO, Section IV.4 step2: Add logic to check for intersection  
  if (leftADC < whiteThreshold && rightADC < whiteThreshold){
    switch(prevIntersection){
      case true:
        prevIntersection = false;
        retVal = false;
      case false:
        prevIntersection = true;
        retVal = true;
    }
  }


  return retVal;
}



/**
 *  Method that turns the robot the amount of the given angle
*/
void turn(int angle){
  chassis.turnFor(-angle, turnSpeed, true);
}

void goStraight(int distanceToWall){

  while (distance > distanceToWall) {
    distanceReading();
    Serial.println(distance);
    chassis.setWheelSpeeds(5,5);
  }
  Serial.println("hi, no while loop");
  chassis.setWheelSpeeds(0,0);

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
  } 
  else {
    return false;
  }
}
 
/**
 * Directs the robot from the hospital to the fire
*/

void robot1HospitalToFire(){


 // turn away from hospital
  turn(180);

  // line follow out of hospital
  while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

  // turn left
  turn(-100);

 // line follow to fire
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

  // turn onto center line
 turn(100);

// go down center line
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

// keep going down center line
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }
 
 // turn right
  turn(100);

  // up to fire area
  while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

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
void robot1StartToFire(){

 robotLocation = FIRE;

 // go up to line
 goStraight(40);

// up to first intersection by start
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

 // turn left
 turn(-100);

// forward to center intersection
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

// turn right
 turn(100);

// up to turn near fire
while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

// turn right
 turn(100);

// up to fire section
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

 // forwards into fire
 continueUntilDone(85, 115);
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
void robot1FireToPeople(){
 // straight a bit, then right
  continueUntilDone(57, 100);
 // pickup time!
 robotState = ROBOT_RESCUE;

}

/**
 * Directs the robot from the rescue zone to the hospital
*/
void robot1PeopleToHospital(){

  // Route to hospital
  
  // turn around
  turn(200);
  // go straight and left
  continueUntilDone(25, -95);
  // leave fire area
  goStraight(30);

  // to first intersection
  while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

 turn(-100);

 // to center intersection
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

 // end of center line
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }
 turn(100);

 // intersection by hospital
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }
 turn(100);

// into hospital
while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }
 goStraight(60);

  // Update location
  robotLocation = HOSPITAL;
  robotState = ROBOT_IDLE;
}

/**
 * When starting at the gate, directs the robot from the gate to the fire
*/
void robot1GateToFire(){
  // into fire area
  while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

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

void robot1FireToGate(){

// Route from fire to gate
  turn(-100);
  continueUntilDone(25, -100);
  goStraight(30);

  // onto fire line
  while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }
 turn(180);

  robotLocation = GATE;
  robotState = ROBOT_IDLE;
}

/**
  * Directs the robot from the first fire to the second fire
 */
void robot1TakeOver(){

  // Route from fire 2 to fire 1
  turn(-100);
  continueUntilDone(25, -100);
  goStraight(30);

  // onto line following
  while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }
 turn(-100);

  // center intersection
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }
 turn(100);

  // intersection by fire 1
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }
 turn(100);

  // up to fire
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

  // into fire
  continueUntilDone(60, 100);
  continueUntilDone(30, -100);
  robotNumber = 2;
}

void robot2StartToFire(){
  // go up to line
  goStraight(30);

  // start line following, first intersection
  while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }
 turn(90);

// center line intersection
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

// hospital intersection 1
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

 // hospital intersection 2
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

  // intersection by fire station
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

  // up to fire
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

  // into fire
  //go straight, turn right
  continueUntilDone(60, 90);
  // go into fire and turn left
  continueUntilDone(30, -90);

  robotLocation = FIRE;

  if (checkForFire()){
  robotState = ROBOT_FIRE;
 }
 else{
  robotState = ROBOT_FLEE;
 }

}

void robot2FireToPeople(){
continueUntilDone(57, 90);
 // pickup time!
 robotLocation = PEOPLE;
 robotState = ROBOT_RESCUE;

}

void robot2PeopleToHospital(){
  // turn 180
  turn(180);
  // straight turn left
  continueUntilDone(30, -90);
  // leave fire section
  goStraight(80);

  // line follow!
  // intersection by fire
  while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

 // hospital intersection
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }
 turn(-90);

 // up to hospital
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

 //into hospital
 goStraight(30);

  robotLocation = HOSPITAL; 
}

void robot2HospitalToFire(){
  // turn 180
  turn(180);
  // leave hospital area
  goStraight(35);

  // line follow
  // hospital intersection
  while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }
 turn(90);

// fire intersection
 while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

  // up to fire
  while (!checkIntersectionEvent()){
    handleLineFollowing(baseSpeed);
 }

  // into fire
   continueUntilDone(60, 90);
  // go into fire and turn left
  continueUntilDone(30, -90);

  robotLocation = FIRE;

  if (checkForFire()){
    robotState = ROBOT_FIRE;
  }

  else{
    robotState = ROBOT_FLEE;
  }


}

/**
 * Calls the methods to direct the robot2 from the fire to the gate
*/
void robot2FireToGate(){
  // turn left
  turn(-90);
  // go straight, turn left
  continueUntilDone(30, -90);
  // go out gate, turn 180
  continueUntilDone(30, 180);
  // change location + state
  robotState = ROBOT_IDLE;
  robotLocation = GATE;
}


/**
 * Calls the methods to bring robot 2 to the fire from the gate
*/

void robot2GateToFire(){
//into fire
  continueUntilDone(30, -90);
  robotLocation = FIRE;
  if (checkForFire()){
  robotState = ROBOT_FIRE;
  }
  else{
  robotState = ROBOT_FLEE;
  }
}

void robot1Drive(){

  switch (robotLocation)
  {
    // If at the fire , go rescue the people
  case FIRE:

    Serial.println("fire tp people time...");
    robot1FireToPeople();
    break;

    // If at the hospital, go to the fire
  case HOSPITAL:
    robot1HospitalToFire();
    break;

    // If at the starting location, go to the fire
  case INITIAL:
    robot1StartToFire();
    Serial.print("robot state? ");
    break;

    // If at the rescue zone, go to the hospital 
  case PEOPLE:
    robot1PeopleToHospital();
    break;

    // If at the gate, drive to the fire
  case GATE:
    robot1GateToFire();
    break;
  default:
    break;
  }
}

void robot2Drive(){
  //LAURA
  switch (robotLocation)
  {
  case FIRE:
    Serial.println("fire tp people time...");
    robot2FireToPeople();
    break;
  case HOSPITAL:
    robot2HospitalToFire();
    break;
  case INITIAL:
    robot2StartToFire();
    Serial.print("robot state? ");
    break;
  case PEOPLE:
    robot2PeopleToHospital();
    break;
  case GATE:
    robot2GateToFire();
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
  3- IF NOT, adjust and retry (check with rangefinder sensor?)
  4- ONCE GRABBED, switch to drive 
  */
  //arm down and go straight 
  servo.writeMicroseconds(SERVO_UP);
  distanceReading();
  Serial.println(distance);
  goStraight(4);
  distanceReading();
  Serial.println(distance);

  servo.writeMicroseconds(SERVO_DOWN);

  distanceReading();

  //robotLocation = PEOPLE;
  //robotState = ROBOT_DRIVE;
  idle();
  Serial.println("people rescued!");

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
  analogWrite(FAN_PIN,FAN_SPEED); //turn on the fan for 700 ms
  Serial.println("Fan is turned on");
  delay(700);
  analogWrite(FAN_PIN,0); //turn off fan 
  Serial.println("Fan is turned off");
  robotState=ROBOT_DRIVE; //after turned off enter robot drive
 } 
 else{
  robotState=ROBOT_FLEE; //if no fire, flee
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

  switch(robotState)
  {
    // Sets the robot into motion
       case ROBOT_DRIVE:
      setLED(LED_PIN_EX1, HIGH);
      setLED(LED_PIN_EX2, HIGH);
      if (robotNumber == 1){
        robot1Drive();
      }
      else{
        robot2Drive();
      }
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
      if (robotNumber == 1){
        robot1FireToGate();
      }
      else{
        robot2FireToGate();
      }
      
      break;

    default:
      break;
  }

}
