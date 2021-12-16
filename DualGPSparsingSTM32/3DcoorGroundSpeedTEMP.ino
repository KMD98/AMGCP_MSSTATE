#include <SoftwareSerial.h>
#include <Wire.h>
// All parameters to parse headings
char c;
byte CK_A = 0, CK_B = 0;
byte incoming_char;
byte ackPacket[100] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int i = 0;
// end of heading parameters initialization
//Create some I2C subroutines variables and parameters
byte data[19] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

SoftwareSerial serial(PA10,PA9);
TwoWire Wire2(PB9,PB8);
void setup() {
  Serial.begin(115200);
  serial.begin(115200);
  Wire2.begin(0x9);
  Wire2.onRequest(requestEvent);
}

void loop() {
  if (serial.available()) {
    incoming_char = serial.read();
    //for the first 4 bytes, we dont need to store it because it's static, just need to make sure it's the message NAV-REPOLSNED
    if (i < 4 && incoming_char == ackPacket[i]) {
      i++;
    }
    // If the first 4 byte is the message NAV-REPOLSNED, i is incremented and this condition will execute and start storing content of message
    else if (i > 3) {
      ackPacket[i] = incoming_char;
      i++;
    }
  }
  // Once the entire message is stored, run checksum algo
  if (i > 99) {
    checksum();
    //set i back to 0
    i = 0;
  }
}

void checksum() {
  CK_A = 0;
  CK_B = 0;
  for (i = 2; i < 98 ; i++) {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }
  //if checksum passed, go ahead and parse for heading, else dont do anything
  if (CK_A == ackPacket[98] && CK_B == ackPacket[99]) {
    // Serial.println("ACK Received! ");
     parsen();
  }
  else {
    // Serial.println("ACK Checksum Failure: ");
  }
  byte ackPacket[100] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
}

void parsen() {
  //Get latitude with msbyte in last element. Value is multipled 10e-7. Decimal need to be moved 7 to the left
  data[0] = (long)ackPacket[24 + 6] ;
  data[1] = (long)ackPacket[25 + 6];
  data[2] = (long)ackPacket[26 + 6];
  data[3] = (long)ackPacket[27 + 6];
  //Get longitude with msbyte in last element. Value is multipled 10e-7. Decimal need to be moved 7 to the left
  data[4] =  (long)ackPacket[28 + 6];
  data[5] = (long)ackPacket[29 + 6];
  data[6] = (long)ackPacket[30 + 6];
  data[7]= (long)ackPacket[31 + 6];
  //Get hMSL with msbyte in last element. Value is in mm. Decimal need to move 3 to the left because value is in mm
  data[8]  =  (long)ackPacket[36 + 6];
  data[9]= (long)ackPacket[37 + 6];
  data[10]= (long)ackPacket[38 + 6];
  data[11]= (long)ackPacket[39 + 6];
  //get GroundSpeed with msbyte in last element. Value is in mm/s. Decimal need to move 3 to the left because value is in mm
  data[12]  =  (long)ackPacket[60 + 6];
  data[13]= (long)ackPacket[61 + 6];
  data[14]= (long)ackPacket[62 + 6];
  data[15]= (long)ackPacket[63 + 6];
  //Print data
  Serial.print("Latitude: ");Serial.println(((data[0]) + (data[1]<<8) + (data[2]<<16) + (data[3]<<24)));
  Serial.print("Longitude: ");Serial.println(((data[4]) + (data[5]<<8) + (data[6]<<16) + (data[7]<<24)));
  Serial.print("hMSL: ");Serial.println(((data[8]) + (data[9]<<8) + (data[10]<<16) + (data[11]<<24)));
  Serial.print("Ground Speed: ");Serial.println(((data[12]) + (data[13]<<8) + (data[14]<<16) + (data[15]<<24)));


}

void requestEvent(){
  Wire2.write(data,19);
}
