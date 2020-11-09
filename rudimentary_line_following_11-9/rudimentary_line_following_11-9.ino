#include <ECE3.h>

#define PWML 40
#define PWMR 39
#define DIRL 29
#define DIRR 30
#define NSLPL 31
#define NSLPR 11
#define ENCODE_L P5_2
#define ENCODE_R P5_0

int bumpers[] = {24, 25, 6, 27, 8, 28};

// manually initialize these using calibration data
uint16_t sensorMinOffset[] = {597, 493, 534, 410, 455, 560, 516, 580}; 
float sensorMaxFactor[] = {1653.0f, 1757.0f, 1716.0f, 1268.0f, 1317.0f, 1690.0f, 1734.0f, 1670.0f}; 

uint16_t rawSensorValues[8];
uint16_t sensorValues[8];

uint8_t baseSpeed = 40;

float motorR = baseSpeed;
float motorL = baseSpeed;

bool finished = false;

void setup() {
  ECE3_Init();
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission, might be useful for testing
  
   //setup button & LED pins
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

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

  delay(500);
  driveMotors(0, 0);
  digitalWrite(NSLPR, HIGH);
  digitalWrite(NSLPL, HIGH);
  digitalWrite(DIRL, LOW);
  digitalWrite(DIRR, LOW);

  digitalWrite(RED_LED, HIGH);
}

void loop() {
  if(checkBumpers()){
    finished = true;
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
  }
  
  if(!finished){
    ECE3_read_IR(rawSensorValues); // read raw sensor values
    scaleSensorValues();
    float location = sensorFusion();
    /*if(location > 0){
      Serial.print(" ");
    }
    Serial.println(location);*/
    motorR =  baseSpeed + baseSpeed * ((location) * .001);
    motorL = baseSpeed + baseSpeed * ((location) *  -.001);
    
    if(motorR < 20){
      motorR = 20;
    }
    if(motorR > 100){
      motorR == 100;
    }
    if(motorL < 20){
      motorL = 20;
    }
    if(motorL > 100){
      motorL = 100;
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
