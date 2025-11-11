/*
    Modified by Eduardo Bruno, 2025.

    A simple demonstration of the new features.
*/
#include <ACS37800_SPI.h>


ACS37800 mySensor; //Create an object of the ACS37800 class

float vRMS = 0.0;
float iRMS = 0.0;
const uint32_t CUSTOMER_ACCESS_CODE = 0x4F70656E; // Substituir pelo código correto



void setup() {

  ACS37800ERR error;

  Serial.begin(115200); // Start serial communication
  mySensor.beginSPI(5); // cs pin assigned

  delay(5000);

  mySensor.readRMS(&vRMS, &iRMS);

  Serial.println("Voltage:");
  Serial.println(vRMS, 2);
  Serial.println("Current:");
  Serial.println(iRMS, 2);


  
  // Set customer access code
  error = mySensor.writeRegister(ACS37800_CUSTOMER_ACCESS_CODE, ACS37800_REGISTER_VOLATILE_2F);

  ACS37800_REGISTER_0F_t store;
  error = mySensor.readRegister(&store.data.all, ACS37800_REGISTER_EEPROM_0F); 
  Serial.print("Reg 0x0F value: 0x");
  Serial.println(store.data.all, HEX);

  // Modify only desired variables
  store.data.bits.bypass_n_en = 0;
  store.data.bits.n = 0xC8;

  error = mySensor.writeRegister(store.data.all, ACS37800_REGISTER_EEPROM_0F);

  error = mySensor.readRegister(&store.data.all, ACS37800_REGISTER_EEPROM_0F); 
  Serial.print("Changed value of Reg 0x0F: 0x");
  Serial.println(store.data.all, HEX);
  

}

void loop(){

  
}
