//librerias ros/////////////////////////////////////////////////////////////////////////////////
//#include <ros.h>
//////////////////////////////////////////////////////////////////////////////////////////////// 

//librerias sensor de peso//////////////////////////////////////////////////////////////////////
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
///////////////////////////////////////////////////////////////////////////////////////////////

//variables globales sensor de peso////////////////////////////////////////////////////////////
//pins:
const int HX711_dout = 4; //mcu > HX711 dout pin
const int HX711_sck = 5; //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;
///////////////////////////////////////////////////////////////////////////////////////////////

//variables globales sensor de seguridad///////////////////////////////////////////////////////
  int trig = 10;
  int eco = 9;
  int buzz =8;
  int duracion;
  int distancia;
///////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  //ros setup///////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////////////////
  
  //pines sensor de seguridad/////////////////////////////////////////////////////////////////////
  pinMode(trig, OUTPUT);
  pinMode(eco, INPUT);
  pinMode(buzz, OUTPUT);
  ////////////////////////////////////////////////////////////////////////////////////////////////

  //setup sensor de peso//////////////////////////////////////////////////////////////////////////
  Serial.begin(57600);
  Serial.println();
  Serial.println("Starting...");

  LoadCell.begin();
  //LoadCell.setReverseOutput(); //uncomment to turn a negative output value to positive
  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(0.1); // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  calibrate(); //start calibration procedure
  //////////////////////////////////////////////////////////////////////////////////////////////
  
}


void loop() {
  sensorultra();
  sensorpeso();
}

void sensorultra(){
  //codigo sensor de seguridad/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  digitalWrite(trig,HIGH);
  delay(1);
  digitalWrite(trig,LOW);
  duracion=pulseIn(eco,HIGH);
  distancia=duracion/58.2;
  delay(200);
  if (distancia>15){
    digitalWrite(buzz,HIGH);
  } else {
    digitalWrite(buzz,LOW);
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}



void sensorpeso(){
  //void loop sensor de peso////////////////////////////////////////////////////////////////////
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      newDataReady = 0;
      t = millis();
    }
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

//Funcion sensor de peso///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate() { 
  LoadCell.update();
  float known_mass = 1;
  LoadCell.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass); //get the new calibration value

#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
}
////////////////////////////////////////////////////////////////////////////////////////////////
