
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

uint16_t rawSensorValues[8];
uint16_t sensorMinOffset[8];
double sensorMaxFactor[8];
uint16_t sensorValues[8];

bool finished = false;

void setup() {
  ECE3_Init();
  
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
    //run calibration
    
    
    
  }else{
    //display results
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
