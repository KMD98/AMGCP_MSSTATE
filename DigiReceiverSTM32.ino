#include<SoftwareSerial.h>
#include<Wire.h>
char received = 0;
char tempCarray [80]; //Declared as much bigger than we need just in case string data gets bigger due to larger coordinate values
SoftwareSerial xbee(PA10, PA9);
TwoWire Wire2(PB7,PB6);
int i = 0;
String temp;
bool newData = true;
int count = -2;
String realData;
byte high;
byte low;
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
          //Serial.print(temp);
          realData = temp;
          Serial.println(realData);
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
  int data_length = realData.length();
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
  else if (count >= 0 and count < data_length and realData != ""){
    Wire2.write(realData[count]);
    count++;
    if (count == data_length){
      count = -2;
      newData = true;
      realData = "";
      high = 0;
      low = 0; 
    }
  }
}
