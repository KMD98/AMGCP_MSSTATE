#include<SoftwareSerial.h>
#include<Wire.h>
char received = 0;
char tempCarray [80]; //Declared as much bigger than we need just in case string data gets bigger due to larger coordinate values
SoftwareSerial xbee(PA10, PA9);
TwoWire Wire2(PB7,PB6);
int count = 0;
byte drone_data[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
struct coor{
  //24 bytes in total to be send
  int left_lat;
  int right_lat;
  int left_lon;
  int right_lon;
  int left_height;
  int right_height;
};
//Declare the instance of the struct
struct coor drone_coor;
void setup() {
  Serial.begin(19200);
  xbee.begin(19200);
  Wire2.begin(0x8);
  Wire2.onRequest(requestEvent);
}

void loop() {
  if (xbee.available() > 0) {
    received = xbee.read();
    if (received == ' '){
      Serial.println("No GPS Data");
    }
    else if (received == '#'){
      tempCarray[count] = received;
      count = 0;
      //Call parsing function to initialize 3D struct
      stringParsing(tempCarray,sizeof(tempCarray));
      //Reset C array
      memset(tempCarray,0,sizeof(tempCarray)); 
    }
    else{
      tempCarray[count] = received;
      count++;
    }
  }
}

void requestEvent(){
  //Write entire byte array
  Wire2.write(drone_data, 12);
}

void stringParsing(char data[], int n){
  int index = 0;
  int comma_place = 0;
  int data_type = 0;
  //if the inital character received is garbage then disregard it and move onto the next
  for(int i = 0; i < n; i++){
    if (data[i] == '!'){
      index = i;
    }
    if (data[i] == ','){
      //Insert parser
      comma_place = i;
      parser(data,sizeof(data),index + 1,comma_place - 1, data_type);
      index = i;
      data_type++;
      if (data_type == 3){
        break;
      }
    }
  }
}

void parser(char data[], int n, int index, int comma_place, int data_type){
  char tempLeft[10];
  char tempRight[10];
  //must clear both temp arrays completely in memory or data will corrupt
  memset(tempLeft,0,sizeof(tempLeft));
  memset(tempRight,0,sizeof(tempRight)); 
  bool isLeft = true;
  int i = 0;
  //Start parsing the numerical values and store to struct
  while(index <= comma_place){
    if (data[index] == '.'){
      isLeft = false;
      index++; //increment past the period to grab the next character
      i = 0; //reset i
    }
    if (isLeft){
      tempLeft[i] = data[index];
    }
    else if (!isLeft){
      tempRight[i] = data[index];
    }
    i++;
    index++;
  }
  switch (data_type){
    case 0:
      drone_coor.left_lat = atoi(tempLeft);
      drone_coor.right_lat = atoi(tempRight);
      convertByte(drone_coor.left_lat, 0);
      convertByte(drone_coor.right_lat,2);
      Serial.print(drone_coor.left_lat);Serial.print(".");Serial.println(drone_coor.right_lat);
      break;
    case 1:
      drone_coor.left_lon = atoi(tempLeft);
      drone_coor.right_lon = atoi(tempRight);
      convertByte(drone_coor.left_lon,4);
      convertByte(drone_coor.right_lon,6);
      Serial.print(drone_coor.left_lon);Serial.print(".");Serial.println(drone_coor.right_lon);
      break;
    case 2:
      drone_coor.left_height = atoi(tempLeft);
      drone_coor.right_height = atoi(tempRight);
      convertByte(drone_coor.left_height,8);
      convertByte(drone_coor.right_height,10);
      Serial.print(drone_coor.left_height);Serial.print(".");Serial.println(drone_coor.right_height);
      break;
  }
}

void convertByte(int data, int n){
  byte high,low;
  high = data>>8;
  low = data & (0xFF);
  drone_data[n] = high;
  drone_data[n+1] = low;
}
