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
  //24 bytes in total to be send
  int left_lat;
  int right_lat;
  int left_lon;
  int right_lon;
  int left_height;
  int right_height;
}
//Declare the instance of the struct
struct 3Dcoor drone_coor;
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
          //Call parsing function to initialize 3D struct
          stringParsing(tempCarray,sizeof(tempCarray));
          //Reset C array
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
  Wire.write((byte *)&drone_coor, sizeof drone_coor);
}

void stringParsing(char data[], int n){
  int index = 0;
  int comma_place = 0;
  //if the inital character received is garbage then disregard it and move onto the next
  for(int i = 0; i < n; i++){
    if (data[i] == '!'){
      index = i
    }
    if (data[i] == ','){
      //Insert parser
      comma_place = i;
      parser(data,sizeof(data),index + 1,comma_place - 1);
      index = i
    }
    else if (data[i] == '#'){
      comma_place = i;
      parser(data,sizeof(data),index + 1, comma_place - 1);
    }
  }
}

void parser(char data[], int n, int index, int comma_place){
  string temp = "";
  while(index < comma_place){
    temp = temp + data[index];
    index++;
  }
}
