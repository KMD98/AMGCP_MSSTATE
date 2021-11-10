#include <SoftwareSerial.h>
#include <Wire.h>
const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };
SoftwareSerial serial(PA10,PA9);
TwoWire Wire2(PB7,PB6);
byte data[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
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
  if ( processGPS() ) {
    // Uncomment code block below to see string parsed result
    /*String latInteger = String(int(posllh.lat/10000000.0f));
    String lonInteger = String(int(posllh.lon/10000000.0f));
    // Convert lat/lon to strings and get the lat and long decimals
    String decimalLat = String(abs(posllh.lat%10000000));
    String decimalLon = String(abs(posllh.lon%10000000));
    String lat = latInteger + "." + decimalLat;
    String lon = lonInteger + "." + decimalLon;
    String height = String(posllh.height/1000.0f);
    String heightMSL = String(posllh.hMSL/1000.0f);
    mgcp_coor = "!" + lat + "," + lon + "," + height + "," + heightMSL + "!";
    Serial.println(mgcp_coor);*/
    //Gather gps coor data and intialize the data array to be sent to I2C
    convertByteArray(0,4);
    convertByteArray(4,8);
    convertByteArray(8,12);
    //The decimal is the seventh place from the right for lat and lon and 3 place from the right for height.
    Serial.print("Latitude from byte array is: ");Serial.print((data[0]<<24) + (data[1]<<16) + (data[2]<<8) + data[3]); Serial.print(" Sign is: "); Serial.println(data[12]);
    Serial.print("Longitude from byte array is: ");Serial.print((data[4]<<24) + (data[5]<<16) + (data[6]<<8) + data[7]);Serial.print(" Sign is: "); Serial.println(data[13]);
    Serial.print("Latitude from byte array is: ");Serial.print((data[8]<<24) + (data[9]<<16) + (data[10]<<8) + data[11]);Serial.print(" Sign is: "); Serial.println(data[14]); 
  }
}

void requestEvent(){
  Wire2.write(data,15);
}

void convertByteArray(int indexer, int num){
  long temp = 0;
  switch (indexer){
    case 0:
      if(posllh.lat < 0){
        data[12] = 1;
        temp = abs(posllh.lat);
      }
      else if (posllh.lat >= 0){
        data[12] = 0;
        temp = posllh.lat;
      }
      break;
    case 4:
      if(posllh.lon <0){
        data[13] = 1;
        temp = abs(posllh.lon);
      }
      else if (posllh.lon >= 0){
        data[13] = 0;
        temp = posllh.lon;
      }
      break;
    case 8:
      if(posllh.height < 0){
        data[14] = 1;
        temp = abs(posllh.height);
      }
      else if(posllh.height >= 0){
        data[14] = 0;
        temp = posllh.height;
      }
      break;
  }
  //Note that lat and lon values from the struct need to be divided by 10000000.0f and the height needs to be divided by 1000.0f on the master side to find the decimal placement.
  int c = 0;
  for(int i = indexer; i < num; i++){
    if (i != num - 1){
      data[i] = temp>>(8*(3-c));         
    }
    else if(i == num - 1){
      data[i] = temp & 0xff;
    }
    c++;
  }  
}
