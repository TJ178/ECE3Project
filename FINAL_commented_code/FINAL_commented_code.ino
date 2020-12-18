/* ECE3 Line Following Project Fall 2020
 * Written by Timothy Jacques and Ellijah Ellsberry
 */

//include library to read IR sensors
#include <ECE3.h>

//pin macros
#define PWML 40
#define PWMR 39
#define DIRL 29
#define DIRR 30
#define NSLPL 31
#define NSLPR 11
#define ENCODE_L P5_2
#define ENCODE_R P5_0
#define BUTTON_R P1_1
#define BUTTON_L P1_4

//tuning constants
#define BASE_SPEED 90
#define MIN_SPEED 20
#define MAX_SPEED 180

#define BLACK_LINE_MAXCOUNT 3  //number of continuous "black" samples required to trigger the line
#define TURN_ENCODER_DIFF 700  //encoder difference goal for 180 degree turn

//line following PID constants
#define KP 0.001
#define KD 0.027

//bumper pin constants
int bumpers[] = {24, 25, 6, 27, 8, 28};

//calibration constants, different for each robot / light conditions
// manually initialize these using your calibration data
float sensorMinOffset[] = {638.0f, 530.0f, 550.0f, 417.0f, 451.0f, 569.0f, 513.0f, 582.0f};
float sensorMaxFactor[] = {1612.0f, 1434.0f, 1550.0f, 1054.0f, 1076.0f, 1672.0f, 1477.0f, 1668.0f};

//program variables
uint16_t rawSensorValues[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t sensorValues[8] = {0, 0, 0, 0, 0, 0, 0, 0};

//derivative PID variables
float lastLocation;
float derivativeError;
long lastDerivativeTime = 0;

//motor PWM variables
float motorR = BASE_SPEED;
float motorL = BASE_SPEED;

//encoder tracker variables
volatile uint32_t left_encoder = 0;
volatile uint32_t right_encoder = 0;

//runtime variables
bool finished = false;
bool hasTurned = false;
uint8_t blackLineCounter = 0;


void setup() {
  //initialize IR sensor library
  ECE3_Init();

  //setup button & LED pins
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  //setup bumper pins
  for (int i = 0; i < 6; i++) {
    pinMode(bumpers[i], INPUT_PULLUP);
  }

  //setup motor pins
  pinMode(DIRL, OUTPUT);
  pinMode(DIRR, OUTPUT);
  pinMode(NSLPL, OUTPUT);
  pinMode(NSLPR, OUTPUT);
  pinMode(ENCODE_R, INPUT);
  pinMode(ENCODE_L, INPUT);
  pinMode(BUTTON_L, INPUT_PULLUP);
  pinMode(BUTTON_R, INPUT_PULLUP);

  //setup encoder tracking interrupts
  attachInterrupt(ENCODE_R, incr_r, FALLING);
  attachInterrupt(ENCODE_L, incr_l, FALLING);

  //wait until button is pressed to calibrate white-level of sensors, update LEDs accordingly
  digitalWrite(RED_LED, HIGH);
  while (digitalRead(BUTTON_L)) {};
  calibrateSensors();
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);

  //wait until button is pressed a second time to start running
  delay(500);
  while (digitalRead(BUTTON_L)) {};

  //initialize motors
  driveMotors(0, 0);
  digitalWrite(NSLPR, HIGH);
  digitalWrite(NSLPL, HIGH);
  digitalWrite(DIRL, LOW);
  digitalWrite(DIRR, LOW);

  //initialize lastLocation variable (for derivative)
  ECE3_read_IR(rawSensorValues);
  scaleSensorValues();
  lastLocation = sensorFusion();

  //update LEDs
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(500);
}


void loop() {

  //check all bumperpins, stop motors if any are pressed
  if (checkBumpers()) {
    finished = true;
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, HIGH);
  }

  if (!finished) {
    //read, scale, and combine new sensor values to get location error
    ECE3_read_IR(rawSensorValues);
    scaleSensorValues();
    float location = sensorFusion();

    //check for black line
    if (sumOfSensors() > 7250) {
      blackLineCounter++;
    } else {
      blackLineCounter = 0;
    }

    //turn if the conditions for a black line have been reached
    if (blackLineCounter > BLACK_LINE_MAXCOUNT) {
      if (!hasTurned) {
        driveMotors(0, 0);
        blackLineCounter = 0;
        turn();
        hasTurned = true; //prevent turning twice

        //go forwards for a bit afterwards (fixes issues with robot not turning all the way & completely missing line)
        driveMotors(BASE_SPEED, BASE_SPEED);
        delay(275);
      } else {

        //throw finished flag if black line has been reached a second time
        finished = true;
      }
    }

    //rudimentary path-loss detection
    if(sumOfSensors() < 250){
      location = lastLocation;
      digitalWrite(YELLOW_LED, HIGH);
    }


    //calculate derivative error
    lastDerivativeTime = millis();
    derivativeError = location - lastLocation;
    lastLocation = location;

    //calculate new motor speeds based on PID constants and error
    motorR = BASE_SPEED + BASE_SPEED * (derivativeError * KD) + BASE_SPEED * location * KP;
    motorL = BASE_SPEED + BASE_SPEED * (derivativeError *  -KD) + BASE_SPEED * location * -KP;

    //limit motor speeds
    if (motorR < MIN_SPEED) {
      motorR = MIN_SPEED;
    }
    if (motorR > MAX_SPEED) {
      motorR == MAX_SPEED;
    }
    if (motorL < MIN_SPEED) {
      motorL = MIN_SPEED;
    }
    if (motorL > MAX_SPEED) {
      motorL = MAX_SPEED;
    }

    //drive motors at new calculated speed
    driveMotors( (int) motorL, (int) motorR);

  } else {

    //if finished flag is thrown, stop motors and wait
    driveMotors(0, 0);

    //wait until button is pressed again to restart running
    while (digitalRead(BUTTON_L)) {};

    //reset runtime variables for new trial
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    delay(500);
    ECE3_read_IR(rawSensorValues);
    scaleSensorValues();
    lastLocation = sensorFusion();
    finished = false;
    hasTurned = false;
    blackLineCounter = 0;
  }
}


void driveMotors(int pwmL, int pwmR) {

  //limit PWM to valid values
  if (pwmL > 255) {
    pwmL = 255;
  } else if (pwmL < 0) {
    pwmL = 0;
  }
  if (pwmR > 255) {
    pwmR = 255;
  } else if (pwmR < 0) {
    pwmR = 0;
  }

  //set motor driver pins to PWM signal
  analogWrite(PWMR, pwmR);
  analogWrite(PWML, pwmL);
}


void turn() {

  //turn motors at a constant speed until the given encoder difference is met
  //allows for consistent turning despite changes in actual motor speed
  resetEncoders();
  digitalWrite(DIRR, HIGH);
  while (left_encoder + right_encoder < TURN_ENCODER_DIFF) {
    driveMotors(100, 100);
  }
  digitalWrite(DIRR, LOW);
  driveMotors(1, 1);
}

//check if any bumpers are pressed
bool checkBumpers() {
  for (int i = 0; i < 6; i++) {
    if (!digitalRead(bumpers[i])) {
      return true;
    }
  }
  return false;
}

void scaleSensorValues() {
  //use calibration values to normalize sensor values
  for (int i = 0; i < 7; i++) {
    sensorValues[i] = ((rawSensorValues[i] - sensorMinOffset[i]) * 1000) / sensorMaxFactor[i]; // normalize output value of each sensor to be between 0 and 1000
  }
}

float sensorFusion() { // 8-4-2-1 weighted total
  return (-(8.0 * sensorValues[0] + 4.0 * sensorValues[1] + 2.0 * sensorValues[2] + 1.0 * sensorValues[3]) + (1.0 * sensorValues[4] + 2.0 * sensorValues[5] + 4.0 * sensorValues[6] + 8.0 * sensorValues[7])) / 4.0f;
}

int sumOfSensors() {
  return sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4] + sensorValues[5] + sensorValues[6] + sensorValues[7];
}

void calibrateSensors() {
  //take 10 readings and update min value of each sensor
  //will adjust white-level to current ambient conditions
  driveMotors(0, 0);
  for (int i = 0; i < 9; i++) {
    ECE3_read_IR(rawSensorValues);
    for (int j = 0; j < 8; j++) {
      if (rawSensorValues[j] < sensorMinOffset[j]) {
        sensorMinOffset[j] = rawSensorValues[j];
      }
    }
  }
}

//encoder interrupt functions
void incr_r() {
  right_encoder++;
}
void incr_l() {
  left_encoder++;
}

void resetEncoders() {
  left_encoder = 0;
  right_encoder = 0;
}
