#include<SoftwareSerial.h>

int received = 0;
char tempCarray[90];
int i;

SoftwareSerial xbee(13,15);
SoftwareSerial rpi(5,4); //D1,D2 (RX,TX) Connect D1 to TX of RPI and D2 to RX of RPI
void setup() {

  Serial.begin(19200);

  xbee.begin(19200);
  rpi.begin(19200);

}

void loop() {

  if (xbee.available() > 0) {

    received = xbee.read();
    if (received == '#'){
      tempCarray[i] = received;
      i = 0;
      int c = 0;
      for(c = 0; c <sizeof(tempCarray); c++){
        Serial.print(tempCarray[c]);
        rpi.write(tempCarray[c]); //write to rpi with TX pin and RPI must take the TX and put it in its RX
      }
      memset(tempCarray,0,sizeof(tempCarray));
      Serial.println();
    }
    else if (received == ' '){
    Serial.println("no Data. Check transmitter power");
    }
    else{
      tempCarray[i] = received;
      i++;
    }
  }
}
