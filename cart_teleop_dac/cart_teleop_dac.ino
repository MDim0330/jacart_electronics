/*
   cart_teleop_new_board.ino - Contains the main arduino code for controlling our autonomous golf cart with MCP4725 DACs.

      Author: Brandon Parr
      Version: 0.5

       11/09/2020: modified by Dick Shimp to include the MCP4725 DAC development boards
       12/9/2020: edited by Jason Forsyth to increase code structure and readability
       1/21/2021: edited by Jason Forsyth to comment out all Serial.print() statements

       4/29/2021: modified LOWER_BRAKE_BOUND to deal with intermittent stopping condition. Also, reduced
       print statement verbosity to report steering, throttle, and braking targets.


       &&&&& NOTE: You must use the Arduino Nano "OLD BOOTLOADER" setting when programming this device!!! Tools -> Processor -> ATMega 328P (Old Bootloader)
*/

#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_MCP4725.h>

int throttle = 5;         /* pin 3 */
int brake = 6;            /* pin 5 */
int throttleRelay = 7;    /* pin 7 */
int brakeRelay = 8;       /* pin 8 */
int wiper = A0;           /* pin A0 */

/* bounds for throttle mapping */
const int LOWER_THROTTLE_BOUNDS = 25;
const int UPPER_THROTTLE_BOUND = 226;

/* bounds for brake mapping */
const int LOWER_BRAKE_BOUND = 21; //adjusted to deal with intermittent braking
const int UPPER_BRAKE_BOUND = 234;

/*bounds for steering mapping */
const int LOWER_STEER_BOUND = 0;
const int UPPER_STEER_BOUND = 4096;

/* empirical potentiometer bounds */
const int LOWER_STEER_POT_BOUND = 310;
const int UPPER_STEER_POT_BOUND = 620;

/* Neutral steer value */
const float NEUTRAL_STEER = 2.5; //voltage to indicate "no turn" steering

/* control signals for steering, break, and throttle */
double steeringTarget = 50; //set steering wheel position to 50% which should be middle
int brakeTarget = 0;
int throttleTarget = 0;

const float STEER_VOLT_MAX_OFFSET = .7;
const float STEER_VOLT_MIN_OFFSET = .3;

/* upper and lower bounds of PID output */
const int LOWER_PID_BOUND = -255;
const int UPPER_PID_BOUND = 255;

/*
   Input: currentSteeringPot
   Output: pidSignal
   Setpoint: steeringTarget
   Kp: 10 (proportional term)
   Ki: 2.0 (integral term)
   Kd: 1 (derivative term)
   Controller Direction: DIRECT
*/

/*Values for the pid*/
double currentSteeringPot = -1;
double smoothedSteeringSignal = -1;
double pidSignal = -1;


const float KP = 10; //proportional gain
const float KI = 2.0; //integral gain
const float KD = 1; //derivative gain
PID myPID(&currentSteeringPot, &pidSignal, &steeringTarget, KP, KI, KD, DIRECT);

/* magic numbers */
const int MAGIC_START = 42; //start byte for incoming messages
const int MAGIC_END = 21; //end byte for incoming messages

/* objects to handle the two DACs on I2c*/
Adafruit_MCP4725 dac1;
Adafruit_MCP4725 dac2;

/* Main program setup */
void setup() {
  // set up serial
  Serial.begin(57600);
  while (!Serial) {
    delay(15);
  }

  //set up DACs
  dac1.begin(0x63); //i2c address for device
  dac2.begin(0x62); //i2c address for device

  // set up relays
  pinMode(throttleRelay, OUTPUT);
  pinMode(brakeRelay, OUTPUT);

  // set up outputs
  pinMode(throttle, OUTPUT);
  pinMode(brake, OUTPUT);

  // set up wiper
  pinMode(wiper, INPUT);

  //Serial.println("STARTING");

  //setup PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(LOWER_PID_BOUND, UPPER_PID_BOUND);
  myPID.SetSampleTime(20);
}

/* Main program loop */

long last_heart_beat = 0;
unsigned int heart_beat_counter = 0;
void loop() {
  delay(1);
  readCommands();
  steer();
  setThrottle();
  setBrake();

  //implement basic heart beat system
  while (abs(millis() - last_heart_beat) > 50)
  {
    Serial.print(steeringTarget);
    Serial.print(",");
    Serial.print(throttleTarget);
    Serial.print(",");
    Serial.println(brakeTarget);


    last_heart_beat = millis();
    heart_beat_counter++;
  }

}

void readCommands() {

  // values read in from serial
  int firstByte = Serial.read();
  int secondByte = Serial.read();
  int throttleCommand = -1;
  int brakeCommand = -1;
  int steeringCommand = -1;


  // check to see if you have gotten the magic numbers.
  if (firstByte == MAGIC_START && secondByte == MAGIC_END) {

    // Read in throttle, brake, and steering data
    throttleCommand = Serial.read();
    brakeCommand = Serial.read();
    steeringCommand = Serial.read();


    if (throttleCommand != -1 && brakeCommand != -1 && steeringCommand != -1) {
      throttleTarget = throttleCommand;
      brakeTarget = brakeCommand;
      steeringTarget = steeringCommand;
      //Serial.print("Steering Target= ");
      //Serial.println(steeringTarget);
    }
  }
}

void setThrottle() {
  int val = map(throttleTarget, 0, 255, LOWER_THROTTLE_BOUNDS, UPPER_THROTTLE_BOUND);
  analogWrite(throttle, val);

  if (throttleTarget == 0) {
    digitalWrite(throttleRelay, LOW);
  } else {
    digitalWrite(throttleRelay, HIGH);
  }
}

void setBrake() {
  int val = map(brakeTarget, 0, 255, LOWER_BRAKE_BOUND, UPPER_BRAKE_BOUND);
  analogWrite(brake, val);

  if (brakeTarget == 255) {
    digitalWrite(brakeRelay, LOW);
  } else {
    digitalWrite(brakeRelay, HIGH);
  }
}

//process steering command
void steer() {
  if (steeringTarget != -1) {
    // read in the potentiometer value and map from 0 (full left) to 100 (full right)
    if (smoothedSteeringSignal == -1)
      smoothedSteeringSignal = analogRead(wiper);
    else {
      smoothedSteeringSignal = smoothedSteeringSignal * .9 + analogRead(wiper) * .1;
    }
    //Serial.print("Pot Absolute Value: ");
    //Serial.println(currentSteeringPot);
    currentSteeringPot = mapf(smoothedSteeringSignal, LOWER_STEER_POT_BOUND, UPPER_STEER_POT_BOUND, 0.0, 100.0);
    //Serial.print("CURR POT ");
    //Serial.println(currentSteeringPot);
    myPID.Compute();
    //Serial.print("RAW COMPUTE ");
    //Serial.println(pidSignal);
    float signal = mapf(pidSignal, LOWER_PID_BOUND, UPPER_PID_BOUND, -STEER_VOLT_MAX_OFFSET, STEER_VOLT_MAX_OFFSET);
    //Serial.print("MAPPED SIGNAL: ");
    //Serial.println(signal);
    setSteerVoltages(signal);
  }
}

/* sends a steeringTarget voltage to both left and right steering channels */
void setSteerVoltages(float signal) {
  // check if values are valid - must be between 2.0 & 3.0 inclusive and sum up to 5.0
  int finalLeft = mapf(NEUTRAL_STEER - signal, 0.0, 5.0, LOWER_STEER_BOUND, UPPER_STEER_BOUND);
  int finalRight = mapf(NEUTRAL_STEER + signal, 0.0, 5.0, LOWER_STEER_BOUND, UPPER_STEER_BOUND);

  //Serial.print("IF LEFT VOLT: ");
  //Serial.println(finalLeft);
  //Serial.print("IF RIGHT VOLT: ");
  //Serial.println(finalRight);

  dac1.setVoltage(finalLeft, false);
  dac2.setVoltage(finalRight, false);
}

/* map function for a float value */
float mapf(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
