#include <Wire.h>
struct 3Dcoor{
  //24 bytes in total to be send
  int left_lat;
  int right_lat;
  int left_lon;
  int right_lon;
  int left_height;
  int right_height;
}
//Initialize instance for drone data
struct 3Dcoor drone_coor;

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(19200);  // start serial for output
}

void loop() {
  Wire.requestFrom(8, 24);    // request 6 bytes from slave device #8
  Wire.readBytes((byte*)&drone_coor, sizeof(drone_coor));
  delay(1000);
}
