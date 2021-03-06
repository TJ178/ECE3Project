/*
 * IR Calibration program
 * Press right button to output calibration values to serial
 * Press left button to pause calibration and view calibrated output in real time
 * 
 * written by Tim Jacques & Elijah Ellsberry for ECE3 on 11/2/2020
 */


#include <ECE3.h>

//define button pins
#define BUTTON_R P1_1
#define BUTTON_L P1_4

//array for raw sensor readings
uint16_t rawSensorValues[8];

//arrays for min / max / avg calculation
int counter = 0;
uint16_t rawSum[8];
uint16_t sensorMinOffset[] = {2500,2500,2500,2500,2500,2500,2500,2500};
double sensorMaxFactor[8];
uint16_t sensorValues[8];

bool dispAdjustedValues = false;

//button debounce timer
long buttonPressMillis = 0;

void setup() {
  Serial.begin(9600);
  
  ECE3_Init();
  
  //setup button & LED pins
  pinMode(BUTTON_R, INPUT_PULLUP);
  pinMode(BUTTON_L, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  delay(500);

  //driveMotors(15, 15);
  digitalWrite(RED_LED, HIGH);
}

void loop() {
  
  ECE3_read_IR(rawSensorValues);

  for(int i = 0; i < 8; i++){
    rawSum[i] += rawSensorValues[i];
  }
  counter++;

  //every 10 readings, take average and compare to min / max
  //write new min max if applicable
  if(counter == 9){
    counter = 0;
    for(int i = 0; i < 8; i++){
      rawSensorValues[i] = rawSum[i] / 10.0;
      rawSum[i] = 0;

      if(!dispAdjustedValues){
        if(rawSensorValues[i] < sensorMinOffset[i]){
          sensorMinOffset[i] = rawSensorValues[i];
        }else if(rawSensorValues[i] - sensorMinOffset[i] > sensorMaxFactor[i]){
          sensorMaxFactor[i] = rawSensorValues[i] - sensorMinOffset[i];
        }
      }
    }

     //either display adjusted values or 
     //raw values
    if(dispAdjustedValues){
      for(int i = 0; i < 8; i++){
        Serial.print((rawSensorValues[i] - sensorMinOffset[i])*1000 / sensorMaxFactor[i]);
        Serial.print('\t');
      }
      Serial.println("");
    }else{
      for(int i = 0; i < 8; i++){
        Serial.print(rawSensorValues[i]);
        Serial.print('\t');
      }
      Serial.println("");
    }
  }

  //button debounce code
  //display results if right button is pressed
  //change view mode from raw input to adjusted input if left is pressed
  if(!digitalRead(BUTTON_R)){
    if(buttonPressMillis == 0){
      buttonPressMillis = millis();
    }else if(millis() - buttonPressMillis > 50){
      dispResults();
      buttonPressMillis = 0;
    }
  }else if(!digitalRead(BUTTON_L)){
    if(buttonPressMillis == 0){
      buttonPressMillis = millis();
    }else if(millis() - buttonPressMillis > 50){
      dispAdjustedValues = !dispAdjustedValues;
      buttonPressMillis = 0;
    }
  }
}


//displays mins & maxes to Serial port
//runs when right button is pressed
void dispResults(){
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  //driveMotors(0, 0);

  Serial.println("MIN:");
  for(int i = 0; i < 8; i++){
    Serial.print(sensorMinOffset[i]);
    Serial.print(" ");
  }

  Serial.println("\nMAX:");
  for(int i = 0; i < 8; i++){
    Serial.print(sensorMaxFactor[i]);
    Serial.print(" ");
  }
  
  delay(10000);
}
