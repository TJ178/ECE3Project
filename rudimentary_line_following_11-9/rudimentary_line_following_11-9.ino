#include <ECE3.h>

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

#define BASE_SPEED 50
#define MIN_SPEED 40
#define MAX_SPEED 150

#define GAIN 0.0002


int bumpers[] = {24, 25, 6, 27, 8, 28};

// manually initialize these using calibration data
uint16_t sensorMinOffset[] = {597, 493, 534, 410, 455, 560, 516, 580}; 
float sensorMaxFactor[] = {1653.0f, 1757.0f, 1716.0f, 1268.0f, 1317.0f, 1690.0f, 1734.0f, 1670.0f}; 


uint16_t rawSensorValues[8];
uint16_t sensorValues[8];

float lastLocation;
float derivativeError;
long lastDerivativeTime = 0;
uint16_t derivativeUpdateRate = 250; //millis between updates


float motorR = BASE_SPEED;
float motorL = BASE_SPEED;

bool finished = false;

void setup() {
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission, might be useful for testing
  
   //setup button & LED pins
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  //setup bumper pins
  for(int i = 0; i < 6; i++){
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

  digitalWrite(RED_LED, HIGH);
  while(digitalRead(BUTTON_L)){};
  calibrateSensors();
  digitalWrite(BLUE_LED, HIGH); 

  delay(100);
  while(digitalRead(BUTTON_L)){};
  
  driveMotors(0, 0);
  digitalWrite(NSLPR, HIGH);
  digitalWrite(NSLPL, HIGH);
  digitalWrite(DIRL, LOW);
  digitalWrite(DIRR, LOW);

  //initialize lastLocation variable
  ECE3_read_IR(rawSensorValues);
  scaleSensorValues();
  lastLocation = sensorFusion();

  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(500);
}

void loop() {
  if(checkBumpers()){
    finished = true;
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
  }
  
  if(!finished){
    ECE3_read_IR(rawSensorValues); // read raw sensor values
    scaleSensorValues();
    float location = sensorFusion();
    /*if(location > 0){
      Serial.print(" ");
    }
    Serial.println(location);*/

    if(lastDerivativeTime - millis() > derivativeUpdateRate){
      lastDerivativeTime = millis();
      derivativeError = location - lastLocation;
      lastLocation = location;
    }


    motorR =  BASE_SPEED + BASE_SPEED * ((location + derivativeError) * GAIN);
    motorL = BASE_SPEED + BASE_SPEED * ((location + derivativeError) *  -GAIN);
    
    if(motorR < MIN_SPEED){
      motorR = MIN_SPEED;
    }
    if(motorR > MAX_SPEED){
      motorR == MAX_SPEED;
    }
    if(motorL < MIN_SPEED){
      motorL = MIN_SPEED;
    }
    if(motorL > MAX_SPEED){
      motorL = MAX_SPEED;
    }
    
    driveMotors( (int) motorL, (int) motorR);
    //Serial.print(motorL);
    //Serial.print('\t');
    //Serial.println(motorR);
  }else{
    driveMotors(0,0);
    delay(1000);
  }
}


void driveMotors(int pwmL, int pwmR){
  if(pwmL > 255){
    pwmL = 255;
  }else if(pwmL < 0){
    pwmL = 0;
  }
  if(pwmR > 255){
    pwmR = 255;
  }else if(pwmR < 0){
    pwmR = 0;
  }
  analogWrite(PWMR, pwmR);
  analogWrite(PWML, pwmL);
}

//check if any bumpers are pressed
bool checkBumpers(){
  for(int i = 0; i < 6; i++){
    if(!digitalRead(bumpers[i])){
      return true;
    }
  }
  return false;
}

void scaleSensorValues(){
  for(int i=0; i<7; i++){ 
    sensorValues[i]=((rawSensorValues[i]-sensorMinOffset[i])*1000)/sensorMaxFactor[i]; // normalize output value of each sensor to be between 0 and 1000
  }
}

float sensorFusion(){ // 8-4-2-1 weighted total
  return (-(8.0*sensorValues[0]+6.0*sensorValues[1]+6.0*sensorValues[2]+3.0*sensorValues[3]) + (3.0*sensorValues[4]+6.0*sensorValues[5]+6.0*sensorValues[6]+8.0*sensorValues[7]))/4.0f; 
}

void calibrateSensors(){
  //take 10 readings and update min value of each sensor
  driveMotors(0,0);
  for(int i = 0; i<9; i++){
    ECE3_read_IR(rawSensorValues);
      for(int j=0; j<8; j++){
        if(rawSensorValues[j] < sensorMinOffset[j]){
        sensorMinOffset[j] = rawSensorValues[j];
      }
    }
  }
}
