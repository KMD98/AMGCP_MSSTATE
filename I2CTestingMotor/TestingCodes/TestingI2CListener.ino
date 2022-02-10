#include <Wire.h>
#include <SoftwareSerial.h>

// LED on pin 13
const int ledPin = PA7; 
byte data[] = {0,0,0,0};
byte requestData[] = {30,10,20,1,12,4,5,6,7,8,2,30};
TwoWire Wire2(PB9,PB8);
SoftwareSerial serial(PA10,PA9);

void setup() {
  Serial.begin(115200);
  serial.begin(115200);
  // Join I2C bus as slave with address 8
  Wire2.begin(0x8);
  
  // Call receiveEvent when data received                
  Wire2.onReceive(receiveEvent);

  //Call request event when data is requested
  Wire2.onRequest(requestEvent);
  
  // Setup pin 13 as output and turn LED off
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
}
 
void loop() {
  delay(100);
}

void requestEvent(){
  Wire2.write(requestData,12);
}

void receiveEvent(int desiredSpeeds) {
  int i = 0;
  Wire2.read(); //Get rid of the 0 delimiter coming from the master. The remaining bytes are the valuable values.
  while (Wire2.available()) { // loop through all data
    data[i] = Wire2.read(); // receive byte as a byte
    i++;
  }
  for(int c = 0; c < 4; c++){
    Serial.print(data[c]);Serial.print(" ");
  }
  Serial.println();
}
