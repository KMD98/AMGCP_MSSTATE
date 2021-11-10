#include <SoftwareSerial.h>
#include <Wire.h>
// All parameters to parse headings
char c;
double heading;
byte CK_A = 0, CK_B = 0;
byte incoming_char;

byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int i = 0;
int turn = 0;
// end of heading parameters initialization
//Create some I2C subroutines variables and parameters
long tempHeading;
byte data[] = {0,0,0,0};

SoftwareSerial serial(PA10,PA9);
TwoWire Wire2(PB9,PB8);
void setup() {
  Serial.begin(115200);
  serial.begin(115200);
  Wire2.begin(0x7);
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
  if (i > 71) {
    checksum();
    //set i back to 0
    i = 0;
  }
}

void checksum() {
  CK_A = 0;
  CK_B = 0;
  for (i = 2; i < 70 ; i++) {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }
  //if checksum passed, go ahead and parse for heading, else dont do anything
  if (CK_A == ackPacket[70] && CK_B == ackPacket[71]) {
    // Serial.println("ACK Received! ");
     parsen();
  }
  else {
    // Serial.println("ACK Checksum Failure: ");
  }
  byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
}

void parsen() {
  heading  =  (long)ackPacket[24 + 6] ;
  heading += (long)ackPacket[25 + 6] << 8;
  heading += (long)ackPacket[26 + 6] << 16 ;
  heading += (long)ackPacket[27 + 6] << 24 ;
  heading = heading / 100000; //get rid of 1e-5 scale based on protocol book
  if (heading > 360) heading = heading - 360;
  if (heading < 0) heading = heading + 360;
  //temp heading is to be sent. Remember to have master divide by 100 to get the actual decimal placement.
  tempHeading = heading * 100;
  data[0] = tempHeading>>24;
  data[1] = tempHeading>>16;
  data[2] = tempHeading>>8;
  data[3] = tempHeading & 0xff;
  // Turn heading double into a string
  Serial.print("Heading : ");Serial.println(((data[0]<<24) + (data[1]<<16) + (data[2]<<8) + data[3])/100.0f);
}

void requestEvent(){
  Wire2.write(data,4);
}
