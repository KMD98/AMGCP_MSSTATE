#include <SoftwareSerial.h>
#include <Wire.h>
const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };
SoftwareSerial serial(PA10,PA9);
TwoWire Wire2(PB7,PB6);
struct NAV_POSLLH {
  // Use char to store 1 bytes class 0x01
  unsigned char cls;
  // Use char to store 1 byte ID 0x02
  unsigned char id;
  // Use short to store 2 byte length value
  unsigned short len;
  //Everything after len is 4 bytes so use a long
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

NAV_POSLLH posllh;

void calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_POSLLH); i++) {
    CK[0] += ((unsigned char*)(&posllh))[i];
    CK[1] += CK[0];
  }
}

bool processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_POSLLH);

  while ( serial.available() ) {
    byte c = serial.read();
    if ( fpos < 2 ) {
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0;
    }
    else {
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&posllh))[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        calcChecksum(checksum);
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return false;
}

// Create some i2c check and routine variables
bool newData = true;
String mgcp_coor;
int count = -2;
byte high;
byte low;

void setup() 
{
  Serial.begin(115200);
  serial.begin(115200);
  Wire2.begin(0x9);
  Wire2.onRequest(requestEvent);
}

void loop() {
  //When finished reading the message (payload finished) print result from the data struct
  if (newData){
    if ( processGPS() ) {
      // Get the lat and lon integer value
      String latInteger = String(int(posllh.lat/10000000.0f));
      String lonInteger = String(int(posllh.lon/10000000.0f));
      // Convert lat/lon to strings and get the lat and long decimals
      String decimalLat = String(abs(posllh.lat%10000000));
      String decimalLon = String(abs(posllh.lon%10000000));
      String lat = latInteger + "." + decimalLat;
      String lon = lonInteger + "." + decimalLon;
      String height = String(posllh.height/1000.0f);
      String heightMSL = String(posllh.hMSL/1000.0f);
      mgcp_coor = "!" + lat + "," + lon + "," + height + "," + heightMSL + "!";
      Serial.println(mgcp_coor);
    }
  }
}

void requestEvent(){
  newData = false;
  int data_length = mgcp_coor.length();
  if (count == -2){
    high = data_length>>8;
    low = data_length & (0xFF);
    Wire2.write(high);
    count++;
  }
  else if (count == -1){
    Wire2.write(low);
    count++;
  }  
  else if (count >= 0 and count < data_length and mgcp_coor !=""){
    Wire2.write(mgcp_coor[count]);
    count++;
    if(count == data_length){
      count = -2;
      newData = true;
      mgcp_coor = "";// reset to prevent sending the same instance of mgcp_coor (as in a non updated mgcp coor)
      high = 0;
      low = 0;
    }
  }
}
