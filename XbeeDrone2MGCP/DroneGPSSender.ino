#include <SoftwareSerial.h>

// Connect the GPS RX/TX to esp8266 GPIO 4 or D2 (RX) and GPIO 5 (TX), which is D1, but we dont actually use GPIO 5 but the library calls for it.
SoftwareSerial serial = SoftwareSerial(4,5);
SoftwareSerial Xbee(13,15);

const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

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
    // Pointer to posllh content by Treating the address of posllh as a byte pointer. Use i as the address indexer in the struct and then grab the content at that address. it is the same as *(address + offset(i))
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

void setup() 
{
  Serial.begin(19200);
  serial.begin(19200);
  Xbee.begin(19200);
}

void loop() {
  //When finished reading the message (payload finished) print result from the data struct
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
    String drone_coor = "!" + lat + "," + lon + "," + height + "," + heightMSL + "#";
    char DroneMessage [drone_coor.length() + 1];
    drone_coor.toCharArray(DroneMessage, drone_coor.length() + 1);
    int i = 0;
    for(i = 0; i< sizeof(DroneMessage); i++){
      Xbee.write(DroneMessage[i]);
      delay(1);
    }
  }
}
