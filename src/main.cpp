/*
 * Lab 02 -- Move it!
 */ 

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>
#include <Chassis.h>


// TODO, Section IV.1 step3: Add line to include Chassis.h

// Sets up the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

// TODO, Section IV.1 step4: Declare the chassis object (with default values)
// TODO, Section IV.2 step8: Adjust parameters to better match actual motion
Chassis chassis(7.2, 1440, 12.7); //13.5 instead of 12.7

// A helper function for debugging
const int LED_PIN_EX1 = 12;
const int LED_PIN_EX2 = 18;  // shown as pin A0 on the Romi board

void setLED(int pin, bool value)
{
  Serial.println("setLED()");
  digitalWrite(pin, value);
}

// Defines the robot states
// TODO, Section IV.3 step1: Define a state for line following
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR, ROBOT_LINING};
ROBOT_STATE robotState = ROBOT_IDLE;

// TODO, Section IV.3 step2: Define a baseSpeed
float baseSpeed = 10.0;
// TODO, Section IV.4 step1: Define a whiteThreshold

// idle() stops the motors
void idle(void)
{
  Serial.println("idle()");
  setLED(LED_PIN_EX1, LOW);
  setLED(LED_PIN_EX2, LOW);

  // TODO, Section IV.1 step6: Uncomment call to chassis.idle() to stop the motors
  chassis.idle();

  //set state to idle
  robotState = ROBOT_IDLE;
  Serial.println("/idle()");
}

/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);
  pinMode(LED_PIN_EX1, OUTPUT);

  // TODO, Section IV.1 step5: Initialize the chassis (which also initializes the motors)
  chassis.init();
  chassis.setMotorPIDcoeffs(4, 0.02);

  // TODO, Section IV.2 step1: Adjust the PID coefficients

  idle();

  // Initializes the IR decoder
  decoder.init();

  Serial.println("/setup()");
}

// A helper command to drive a set distance
// At the start, it will take no arguments and we'll hardcode a motion
// TODO, Section IV.2 step4 (but not before!): Edit the function definition to accept a speed
void drive(float speed)
{
  setLED(LED_PIN_EX1, HIGH);
  robotState = ROBOT_DRIVE_FOR;

  // TODO: In Section IV.1 step7 and IV.2 step2, add a call to chassis.setWheelSpeeds() to set the wheel speeds
  chassis.setWheelSpeeds(speed,speed);

  // TODO: In Section IV.2 step9, (optional) temporarily remove the call to setWheelSpeeds() and add a call to chassis.driveFor()

}

// A helper function to turn a set angle
void turn(float ang, float speed)
{
  setLED(LED_PIN_EX2, HIGH);
  robotState = ROBOT_DRIVE_FOR;

  // TODO, Section IV.2 step5: Make a call to chassis.turnFor()
  chassis.turnFor(ang, speed, true);
}

void handleLineFollowing(float baseSpeed)
{
  // Serial.println("basespeed:  " + String(baseSpeed));
  // TODO, Section IV.3 step5: Add line following control
  int16_t leftADC = analogRead(LEFT_LINE_SENSE);
  int16_t rightADC = analogRead(RIGHT_LINE_SENSE);
  // Serial.println("leftADC: " + String(leftADC));
  // Serial.println("rightADC: " + String(rightADC));

  int16_t error = rightADC - leftADC;
  float turnEffort = error * 0.17; //edit k_p!
  chassis.setTwist(baseSpeed, turnEffort);

}

bool checkIntersectionEvent(uint16_t whiteThreshold)
{
  static bool prevIntersection = false;

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
        Serial.println("leftADC: " + String(leftADC));
        Serial.println("rightADC: " + String(rightADC));
        prevIntersection = true;
        retVal = true;
    }
  }

  return retVal;
}

void handleIntersection(void)
{
  Serial.println("Intersection!");

  // TODO, Section IV.4 step4: add a line to drive forward to center the robot
  robotState = ROBOT_DRIVE_FOR;
  Serial.println("about to drive!");
  chassis.driveFor(7, 15, true);
  Serial.println("driving forwards!");

  robotState = ROBOT_IDLE;


}

// TODO, Section IV.2 step7: Declare function handleMotionComplete(), which calls idle()
void handleMotionComplete(){
  idle();
}

void beginLineFollowing(void){
  robotState = ROBOT_LINING;
  setLED(LED_PIN_EX1, HIGH);
  setLED(LED_PIN_EX2, HIGH);
}

// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));

  // TODO, Section IV.1 step1: add "emergency stop"

  if (keyPress == ENTER_SAVE){
    Serial.println("STOP");
    idle();
  }

  switch(robotState)
  {
    case ROBOT_IDLE:
      // TODO, Section IV.1 step2: Handle up arrow button
      if (keyPress == UP_ARROW){
        drive(30);
      }

      // TODO, Section IV.2 step3: Handle the down, left and right arrow buttons
      if (keyPress == DOWN_ARROW){
        drive(-30);
      }

      if (keyPress == RIGHT_ARROW){
        turn(-90,400);
      }

      if (keyPress == LEFT_ARROW){
        turn(90, 400);
      }

      // TODO, Section IV.3 step3: Respond to PLAY_PAUSE press
      if (keyPress == PLAY_PAUSE){
        beginLineFollowing();
      }

      break;
    // TODO, Section IV.5 step1: respond to speed +/- commands (when in ROBOT_LINING state)
    // Use the VOLplus and VOLminus keys on your IR remote
    case ROBOT_LINING:
      Serial.println(keyPress);
      if (keyPress == VOLplus){
        Serial.println("SPEED");
        baseSpeed += 10;
      }
      if (keyPress == VOLminus){
        baseSpeed -= 10;
      }
    default:
      break;
  }
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop()
{
  // Checks for a key press on the remote
  int16_t keyPress = decoder.getKeyCode();
  if(keyPress >= 0) handleKeyPress(keyPress);

  // A basic state machine
  switch(robotState)
  {
    case ROBOT_DRIVE_FOR: 

      // TODO, Section IV.2 step6: Uncomment to handle completed motion
      if(chassis.checkMotionComplete()) handleMotionComplete(); 
      break;

    // TODO, Section IV.3 step4: Add a case to handle line following
    case ROBOT_LINING:
      handleLineFollowing(baseSpeed);
    // TODO, Section IV.4 step3: check/handle intersection
      if(checkIntersectionEvent(50)){
        handleIntersection();
      }
    default:
      break;
  }
}


