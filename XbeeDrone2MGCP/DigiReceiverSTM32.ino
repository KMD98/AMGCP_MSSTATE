#include<SoftwareSerial.h>
#include<Wire.h>
char received = 0;
char tempCarray [80]; //Declared as much bigger than we need just in case string data gets bigger due to larger coordinate values
SoftwareSerial xbee(PA10, PA9);
TwoWire Wire2(PB7,PB6);
int i = 0;
String temp;
bool newData = true;
struct 3Dcoor{
  int left_lat;
  int right_lat;
  int left_lon;
  int right_lon;
  int left_height;
  int right_height;
}
void setup() {
  Serial.begin(19200);
  xbee.begin(19200);
  Wire2.begin(0x8);
  Wire2.onRequest(requestEvent);
}

void loop() {
  if (newData){
      if (xbee.available() > 0) {
        received = xbee.read();
        if (received == ' '){
          Serial.println("No GPS Data");
        }
        else if (received == '#'){
          tempCarray[i] = received;
          i = 0;
          int c = 1;
          // We use a string because there are more library for string. Dealing with a char array is time consuming. We sacrifice a bit of performance but shouldnt matter
          temp = String(tempCarray[0]);
          //if the inital character received is garbage then disregard it and move onto the next
          if (temp != "!"){
            c = 2;
            temp = String(tempCarray[1]);
          }
          //Print the first character to serial com
          while (c < sizeof(tempCarray)){
            temp = temp + String(tempCarray[c]);
            c++;
          }
          //reset all temp variables
          temp = "";
          memset(tempCarray,0,sizeof(tempCarray)); 
        }
        else{
          tempCarray[i] = received;
          i++;
        }
    }
  }

}

void requestEvent(){
  newData = false;
  //Write struct 3D, entire byte array
}
