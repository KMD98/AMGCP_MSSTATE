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
long temp_lat;
long temp_lon;
long temp_hMSL;
long temp_gs;
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
  temp_lon = (long)ackPacket[24 + 6] + ((long)ackPacket[25 + 6]<<8) + ((long)ackPacket[26 + 6]<<16) + ((long)ackPacket[27 + 6]<<24);
  //Get longitude with msbyte in last element. Value is multipled 10e-7. Decimal need to be moved 7 to the left
  temp_lat = (long)ackPacket[28 + 6] + ((long)ackPacket[29 + 6]<<8) + ((long)ackPacket[30 + 6]<<16) + ((long)ackPacket[31 + 6]<<24);
  //Get hMSL with msbyte in last element. Value is in mm. Decimal need to move 3 to the left because value is in mm
  temp_hMSL = (long)ackPacket[36 + 6] + ((long)ackPacket[37 + 6]<<8) + ((long)ackPacket[38 + 6]<<16) + ((long)ackPacket[39 + 6]<<24);
  //get GroundSpeed with msbyte in last element. Value is in mm/s. Decimal need to move 3 to the left because value is in mm
  temp_gs = (long)ackPacket[60 + 6] + ((long)ackPacket[61 + 6]<<8) + ((long)ackPacket[62 + 6]<<16) + ((long)ackPacket[63 + 6]<<24);
  if (temp_lat < 0){
    data[16] = 1;
  }
  else if (temp_lat >= 0){
    data[16] = 0;
  }
  if (temp_lon < 0){
    data[17] = 1;
  }
  else if (temp_lon >= 0){
    data[17] = 0;
  }
  if (temp_hMSL < 0){
    data[18] = 1;
  }
  else if (temp_hMSL >= 0){
    data[18] = 0;
  }
  temp_lat = abs(temp_lat);
  temp_lon = abs(temp_lon);
  temp_hMSL = abs(temp_hMSL);
  //Initialize I2C data array
  data[0] = temp_lat >> 24;
  data[1] = temp_lat >> 16;
  data[2] = temp_lat >> 8;
  data[3] = temp_lat & 0xff;
  data[4] = temp_lon >> 24;
  data[5] = temp_lon >> 16;
  data[6] = temp_lon >> 8;
  data[7] = temp_lon & 0xff;
  data[8] = temp_hMSL >> 24;
  data[9] = temp_hMSL >> 16;
  data[10] = temp_hMSL >> 8;
  data[11] = temp_hMSL & 0xff;
  data[12]= temp_gs >> 24;
  data[13]= temp_gs >> 16;
  data[14]= temp_gs >> 8;
  data[15]= temp_gs & 0xff;
  //Print data for debugging, uncomment to see. Make sure to comment out before deployment.
  /*Serial.print("Latitude: ");Serial.print(((data[0]<<24) + (data[1]<<16) + (data[2]<<8) + data[3]));Serial.print(" The sign is: ");Serial.println(data[16]);
  Serial.print("Longitude: ");Serial.print(((data[4]<<24) + (data[5]<<16) + (data[6]<<8) + data[7]));Serial.print(" The sign is: ");Serial.println(data[17]);
  Serial.print("hMSL: ");Serial.print(((data[8]<<24) + (data[9]<<16) + (data[10]<<8) + data[11]));Serial.print(" The sign is: ");Serial.println(data[18]);
  Serial.print("Ground Speed: ");Serial.println(((data[12]<<24) + (data[13]<<16) + (data[14]<<8) + data[15]));*/


}

void requestEvent(){
  Wire2.write(data,19);
}
