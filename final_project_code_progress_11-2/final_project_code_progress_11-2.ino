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
uint16_t sensorMinOffset[] = {0,0,0,0,0,0,0,0}; 
double sensorMaxFactor[] = {0,0,0,0,0,0,0,0}; 

uint16_t rawSensorValues[8];
uint16_t sensorValues[8];

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

}

void loop() {
  if(!finished){
  ECE3_read_IR(rawSensorValues); // read raw sensor values
  scaleSensorValues();
  sensorFusion();
  }
  else{
     
  }
}


void driveMotors(int pwmL, int pwmR){
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

double sensorFusion(){ // 8-4-2-1 weighted total
  return (-8*sensorValues[0]-4*sensorValues[1]-2*sensorValues[2]-sensorValues[3]+sensorValues[4]+2*sensorValues[5]+4*sensorValues[6]+8*sensorValues[7])/4; 
}
