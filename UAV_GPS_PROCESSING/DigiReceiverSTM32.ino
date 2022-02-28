#include<SoftwareSerial.h>
#include<Wire.h>
char received = 0;
char tempCarray [80]; //Declared as much bigger than we need just in case string data gets bigger due to larger coordinate values
SoftwareSerial xbee(PA10, PA9);
TwoWire Wire2(PB7,PB6);
int count = 0;
byte drone_data[19] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
volatile byte high,low;
struct coor{
  //19 bytes in total to be send
  int left_lat;
  int right_lat;
  int right_lat2;
  int left_lon;
  int right_lon;
  int right_lon2;
  int left_height;
  int right_height;
  byte lat_sign;
  byte lon_sign;
  byte height_sign;
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
      for(int i = 0; i < sizeof(tempCarray);i++){
        Serial.print(tempCarray[i]);
      }
      Serial.println();
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
  Wire2.write(drone_data, 19);
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
  char tempRight2[10];
  //must clear both temp arrays completely in memory or data will corrupt
  memset(tempLeft,0,sizeof(tempLeft));
  memset(tempRight,0,sizeof(tempRight));
  memset(tempRight2,0,sizeof(tempRight2));
  bool isLeft = true;
  int i = 0;
  int decimal_counter = 0;
  //Start parsing the numerical values and store to struct
  while(index <= comma_place){
    if (data[index] == '.'){
      isLeft = false;
      index++; //increment past the period to grab the next character
      i = 0; //reset i
    }
    if (decimal_counter == 3){
      i = 0; //reset i
    }
    if (isLeft){
      tempLeft[i] = data[index];
    }
    else if (!isLeft && data_type == 2){
      tempRight[i] = data[index];
    }
    else if (!isLeft && data_type != 2 && decimal_counter < 3){
      tempRight[i] = data[index];
      decimal_counter++;
    }
    else if(!isLeft && data_type != 2 && decimal_counter >=3){
      tempRight2[i] = data[index];
      decimal_counter++;
    }
    i++;
    index++;
  }
  //Use to find out if negative or positive before storing into struct
  int temp_lat = 0;
  int temp_lon = 0;
  int temp_height = 0;
  switch (data_type){
    case 0:
      temp_lat = atoi(tempLeft);
      if (temp_lat < 0){
        drone_coor.lat_sign = 1;
      }
      else if (temp_lat >= 0){
        drone_coor.lat_sign = 0;
      }
      drone_coor.left_lat = abs(temp_lat);
      drone_coor.right_lat = atoi(tempRight);
      drone_coor.right_lat2 = atoi(tempRight2);
      convertByte(drone_coor.left_lat, 0);
      convertByte(drone_coor.right_lat,2);
      convertByte(drone_coor.right_lat2,4);
      drone_data[16] = drone_coor.lat_sign;
      // the actual bytes from byte array being sent over I2C, the printed values are converted back to ints from byte array.
      Serial.print((drone_data[0]<<8) + drone_data[1]);Serial.print(".");Serial.print((drone_data[2] <<8) + drone_data[3]);Serial.print((drone_data[4]<<8) + drone_data[5]);Serial.print(" Sign lat: ");Serial.println(drone_data[16]);
      break;
    case 1:
      temp_lon = atoi(tempLeft);
      if (temp_lon < 0){
        drone_coor.lon_sign = 1;
      }
      else if (temp_lon >= 0){
        drone_coor.lon_sign = 0;
      }
      drone_coor.left_lon = abs(temp_lon);
      drone_coor.right_lon = atoi(tempRight);
      drone_coor.right_lon2 = atoi(tempRight2);
      convertByte(drone_coor.left_lon,6);
      convertByte(drone_coor.right_lon,8);
      convertByte(drone_coor.right_lon2,10);
      drone_data[17] = drone_coor.lon_sign;
      Serial.print((drone_data[6]<<8) + drone_data[7]);Serial.print(".");Serial.print((drone_data[8] <<8) + drone_data[9]);Serial.print((drone_data[10]<<8) + drone_data[11]);Serial.print(" Sign lon: ");Serial.println(drone_data[17]); ;
      break;
    case 2:
      temp_height = atoi(tempLeft);
      if (temp_height < 0){
        drone_coor.height_sign = 1;
      }
      else if (temp_height >= 0){
        drone_coor.height_sign = 0;
      }
      drone_coor.left_height = abs(temp_height);
      drone_coor.right_height = atoi(tempRight);
      convertByte(drone_coor.left_height,12);
      convertByte(drone_coor.right_height,14);
      drone_data[18] = drone_coor.height_sign;
      Serial.print((drone_data[12]<<8) + drone_data[13]);Serial.print(".");Serial.print((drone_data[14] <<8) + drone_data[15]);Serial.print(" Sign height: "); Serial.println(drone_data[18]);
      break;
  }
}

void convertByte(int data, int n){
  high = data>>8;
  low = data & (0xFF);
  drone_data[n] = high;
  drone_data[n+1] = low;
  high = 0;
  low = 0;
}
