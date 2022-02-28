#include <Wire.h>
#include <SoftwareSerial.h>

// LED on pin 13
const int ledPin = PA7; 
byte requestData[] = {30,10,20,1,12,4,5,6,7,8,2,30};
TwoWire Wire2(PB9,PB8);
SoftwareSerial serial(PA10,PA9);

void setup() {
  Serial.begin(115200);
  serial.begin(115200);
  // Join I2C bus as slave with address 8
  Wire2.begin(0x9);
  

  //Call request event when data is requested
  Wire2.onRequest(requestEvent);
  
  // Setup pin 13 as output and turn LED off
}
 
void loop() {
  delay(1);
}

void requestEvent(){
  Wire2.write(requestData,12);
}
