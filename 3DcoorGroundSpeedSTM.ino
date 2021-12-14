#include <SoftwareSerial.h>
#include <Wire.h>
// All parameters to parse headings
byte CK_A = 0, CK_B = 0;
byte incoming_char;
byte ackPacketPVT[100] = {0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte ackPacketRELPOSNED[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int i = 0;
int counter = 0;
double heading;
long tempHeading;
// end of heading parameters initialization
//Create some I2C subroutines variables and parameters
byte data[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

SoftwareSerial serial(PA10,PA9);
TwoWire Wire2(PB9,PB8);
void setup() {
  Serial.begin(115200);
  serial.begin(115200);
  Wire2.begin(0x7);
  Wire2.onRequest(requestEvent);
}

void loop() {
  if (counter == 0){
    if (serial.available()) {
      incoming_char = serial.read();
      //for the first 4 bytes, we dont need to store it because it's static, just need to make sure it's the message NAV-REPOLSNED
      if (i < 4 && incoming_char == ackPacketPVT[i]) {
        i++;
      }
      // If the first 4 byte is the message NAV-REPOLSNED, i is incremented and this condition will execute and start storing content of message
      else if (i > 3) {
        ackPacketPVT[i] = incoming_char;
        i++;
      }
    }
    // Once the entire message is stored, run checksum algo
    if (i > 99) {
      checksumPVT();
      //set i back to 0
      i = 0;
      counter = 1; //Set to 1 to processing heading info from RELPOSNED
    }
  }
  else if (counter == 1){
    if (serial.available()) {
      incoming_char = serial.read();
      //for the first 4 bytes, we dont need to store it because it's static, just need to make sure it's the message NAV-REPOLSNED
      if (i < 4 && incoming_char == ackPacketRELPOSNED[i]) {
        i++;
      }
      // If the first 4 byte is the message NAV-REPOLSNED, i is incremented and this condition will execute and start storing content of message
      else if (i > 3) {
        ackPacketRELPOSNED[i] = incoming_char;
        i++;
      }
    }
    // Once the entire message is stored, run checksum algo
    if (i > 71) {
      checksumRELPOSNED();
      //set i back to 0
      i = 0;
      counter = 0;
    }    
  }
}

void checksumPVT() {
  CK_A = 0;
  CK_B = 0;
  for (i = 2; i < 98 ; i++) {
    CK_A = CK_A + ackPacketPVT[i];
    CK_B = CK_B + CK_A;
  }
  //if checksum passed, go ahead and parse for heading, else dont do anything
  if (CK_A == ackPacketPVT[98] && CK_B == ackPacketPVT[99]) {
    // Serial.println("ACK Received! ");
     parsenPVT();
  }
  else {
    // Serial.println("ACK Checksum Failure: ");
  }
  byte ackPacketPVT[100] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
}

void checksumRELPOSNED() {
  CK_A = 0;
  CK_B = 0;
  for (i = 2; i < 70 ; i++) {
    CK_A = CK_A + ackPacketRELPOSNED[i];
    CK_B = CK_B + CK_A;
  }
  //if checksum passed, go ahead and parse for heading, else dont do anything
  if (CK_A == ackPacketRELPOSNED[70] && CK_B == ackPacketRELPOSNED[71]) {
    // Serial.println("ACK Received! ");
     parsenRELPOSNED();
  }
  else {
    // Serial.println("ACK Checksum Failure: ");
  }
  byte ackPacketRELPOSNED[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
}
void parsenPVT() {
  //Get latitude with msbyte in last element. Value is multipled 10e-7. Decimal need to be moved 7 to the left
  data[0] = (long)ackPacketPVT[24 + 6] ;
  data[1] = (long)ackPacketPVT[25 + 6];
  data[2] = (long)ackPacketPVT[26 + 6];
  data[3] = (long)ackPacketPVT[27 + 6];
  //Get longitude with msbyte in last element. Value is multipled 10e-7. Decimal need to be moved 7 to the left
  data[4] =  (long)ackPacketPVT[28 + 6];
  data[5] = (long)ackPacketPVT[29 + 6];
  data[6] = (long)ackPacketPVT[30 + 6];
  data[7]= (long)ackPacketPVT[31 + 6];
  //Get hMSL with msbyte in last element. Value is in mm. Decimal need to move 3 to the left because value is in mm
  data[8]  =  (long)ackPacketPVT[36 + 6];
  data[9]= (long)ackPacketPVT[37 + 6];
  data[10]= (long)ackPacketPVT[38 + 6];
  data[11]= (long)ackPacketPVT[39 + 6];
  //get GroundSpeed with msbyte in last element. Value is in mm/s. Decimal need to move 3 to the left because value is in mm
  data[12]  =  (long)ackPacketPVT[60 + 6];
  data[13]= (long)ackPacketPVT[61 + 6];
  data[14]= (long)ackPacketPVT[62 + 6];
  data[15]= (long)ackPacketPVT[63 + 6];
  //Print data
  Serial.print("Latitude: ");Serial.println(((data[0]) + (data[1]<<8) + (data[2]<<16) + (data[3]<<24)));
  Serial.print("Longitude: ");Serial.println(((data[4]) + (data[5]<<8) + (data[6]<<16) + (data[7]<<24)));
  Serial.print("hMSL: ");Serial.println(((data[8]) + (data[9]<<8) + (data[10]<<16) + (data[11]<<24)));
  Serial.print("Ground Speed: ");Serial.println(((data[12]) + (data[13]<<8) + (data[14]<<16) + (data[15]<<24)));
}

void parsenRELPOSNED() {
  heading  =  (long)ackPacketRELPOSNED[24 + 6] ;
  heading += (long)ackPacketRELPOSNED[25 + 6] << 8;
  heading += (long)ackPacketRELPOSNED[26 + 6] << 16 ;
  heading += (long)ackPacketRELPOSNED[27 + 6] << 24 ;
  heading = heading / 100000; //get rid of 1e-5 scale based on protocol book
  if (heading > 360) heading = heading - 360;
  if (heading < 0) heading = heading + 360;
  //temp heading is to be sent. Remember to have master divide by 100 to get the actual decimal placement.
  tempHeading = heading * 100;
  data[16] = tempHeading>>24;
  data[17] = tempHeading>>16;
  data[18] = tempHeading>>8;
  data[19] = tempHeading & 0xff;
  // Turn heading double into a string
  Serial.print("Heading : ");Serial.println(((data[0]<<24) + (data[1]<<16) + (data[2]<<8) + data[3])/100.0f);
}
void requestEvent(){
  Wire2.write(data,20);
}
