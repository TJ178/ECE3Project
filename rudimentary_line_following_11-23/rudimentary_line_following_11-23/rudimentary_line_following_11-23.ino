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

#define BASE_SPEED 75
#define MIN_SPEED 20
#define MAX_SPEED 150

#define TURN_ENCODER_DIFF 715

#define KP 0.00100
#define KD 0.01000


int bumpers[] = {24, 25, 6, 27, 8, 28};

// manually initialize these using your calibration data
float sensorMinOffset[] = {765.0f,714.4f,629.0f,629.0f,697.0f,765.0f,697.0f,997.0f}; 
float sensorMaxFactor[] = {1735.0f,1785.6f,1871.0f,1368.0f,1627.4f,1735.0f,1803.0f,1503.0f};

uint16_t rawSensorValues[8]={0,0,0,0,0,0,0,0};
uint16_t sensorValues[8]={0,0,0,0,0,0,0,0};

float lastLocation;
float derivativeError;
long lastDerivativeTime = 0;
uint16_t derivativeUpdateRate = 250; //millis between updates
int sensorSum = 0;

float motorR = BASE_SPEED;
float motorL = BASE_SPEED;

//encoder tracker variables
volatile uint32_t left_encoder = 0;
volatile uint32_t right_encoder = 0;

bool finished = false;
bool hasTurned=false;

uint8_t blackLineCounter = 0;

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

  //setup encoder tracking interrupts
  attachInterrupt(ENCODE_R, incr_r, FALLING);
  attachInterrupt(ENCODE_L, incr_l, FALLING);

  digitalWrite(RED_LED, HIGH);
  while(digitalRead(BUTTON_L)){};
  calibrateSensors();
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, HIGH); 

  delay(500);
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
  sensorSum=0;
  if(checkBumpers()){
    finished = true;
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RED_LED, HIGH);
  }
  
  if(!finished){
    ECE3_read_IR(rawSensorValues); // read raw sensor values
    scaleSensorValues();
    float location = sensorFusion();
    sensorSum = sumOfSensors();
    if(sensorSum > 6500){
      blackLineCounter++;
    }
    else{
      blackLineCounter=0;
    }
    if(blackLineCounter > 3){
      if(!hasTurned){
        driveMotors(0,0);
      blackLineCounter=0;
      turn();
      hasTurned = true;
      }
      else{
        finished=true;
      }
    }
    
    
    /*
     * if(location > 0){
      Serial.print(" ");
    }
    Serial.println(location);
    */
    
    lastDerivativeTime = millis();
    derivativeError = location - lastLocation;
    lastLocation = location;
    motorR = BASE_SPEED + BASE_SPEED*(derivativeError * KD) + BASE_SPEED*location*KP;
    motorL = BASE_SPEED + BASE_SPEED*(derivativeError *  -KD) + BASE_SPEED*location*-KP;
    
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
    
    //restart running if button is pressed after stopping
    while(digitalRead(BUTTON_L)){};
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    delay(500);
    ECE3_read_IR(rawSensorValues);
    scaleSensorValues();
    lastLocation = sensorFusion();
    finished = false;
    blackLineCounter = 0;
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

void turn(){
  resetEncoders();
  digitalWrite(DIRR, HIGH);
  while(left_encoder + right_encoder < TURN_ENCODER_DIFF){
    driveMotors(50, 50);
  }
  digitalWrite(DIRR, LOW);
  driveMotors(0,0);
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
  return (-(8.0*sensorValues[0]+4.0*sensorValues[1]+2.0*sensorValues[2]+1.0*sensorValues[3]) + (1.0*sensorValues[4]+2.0*sensorValues[5]+4.0*sensorValues[6]+8.0*sensorValues[7]))/4.0f; 
}

int sumOfSensors(){
  //int sum = 0;
//  for(int i=0; i<8; i++){
//    sum += sensorValues[i];
//  }
  return sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4]+sensorValues[5]+sensorValues[6]+sensorValues[7];
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

//encoder interrupt functions
void incr_r(){
  right_encoder++;
}
void incr_l() {
  left_encoder++;
}

void resetEncoders(){
  left_encoder = 0;
  right_encoder = 0;
}
