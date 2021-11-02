#include <Wire.h>
struct coor{
  //24 bytes in total to be send
  int left_lat;
  int right_lat;
  int left_lon;
  int right_lon;
  int left_height;
  int right_height;
};
//Initialize instance for drone data
struct coor drone_coor;
struct coor MGCP_coor;
void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(19200);  // start serial for output
}

void loop() {
  Wire.requestFrom(8, 24);    // request 24 bytes from slave device #8 Drone 3d coor
  Wire.readBytes((byte*)&drone_coor, sizeof(drone_coor));
  Serial.print(drone_coor.left_lat);Serial.print(".");Serial.println(drone_coor.right_lat);
  Serial.print(drone_coor.left_lon);Serial.print(".");Serial.println(drone_coor.right_lon);
  Serial.print(drone_coor.left_height);Serial.print(".");Serial.println(drone_coor.right_height);
  /*Wire.requestFrom(9,24); //Request 24 bytes from salve device #9 MGCP 3d coor
  Wire.readBytes((byte*)&MGCP_coor, sizeof(MGCP_coor));*/
  delay(1000);
}
