//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <Wire.h>

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10,11);	
RoboClaw roboclaw(&serial,10000);

#define address1 0x80
#define address2 0x81

//I2C globals
byte targetRPM[] = {0,0,0,0}; //{driver side,driver direction 1(backwards) or 0 (forward), passenger, passenger direction} where 1 is CW and 0 is CCW
int cmd_vel[] = {0,0}; //{driver side, driver direction(- for forward, + for backwards), passenger, passenger dir}
int previous_cmd[] = {0,0};
void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(9600);
  roboclaw.begin(115200);
  Wire.begin(0x6);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);//Must be present so master can detect slave. Else i2c will bug.
}

void displayspeed(void)
{
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  
  int32_t enc1= roboclaw.ReadEncM1(address1, &status1, &valid1);
  int32_t enc2 = roboclaw.ReadEncM2(address1, &status2, &valid2);
  int32_t speed1 = roboclaw.ReadSpeedM1(address1, &status3, &valid3);
  int32_t speed2 = roboclaw.ReadSpeedM2(address1, &status4, &valid4);
  Serial.print("Encoder1:");
  if(valid1){
    Serial.print(enc1,HEX);
    Serial.print(" ");
    Serial.print(status1,HEX);
    Serial.print(" ");
  }
  else{
    Serial.print("invalid ");
  }
  Serial.print("Encoder2:");
  if(valid2){
    Serial.print(enc2,HEX);
    Serial.print(" ");
    Serial.print(status2,HEX);
    Serial.print(" ");
  }
  else{
    Serial.print("invalid ");
  }
  Serial.print("Speed1:");
  if(valid3){
    Serial.print(speed1,HEX);
    Serial.print(" ");
  }
  else{
    Serial.print("invalid ");
  }
  Serial.print("Speed2:");
  if(valid4){
    Serial.print(speed2,HEX);
    Serial.print(" ");
  }
  else{
    Serial.print("invalid ");
  }
  Serial.println();
}

void loop() {
  if(targetRPM[1] == 0){
    cmd_vel[0] = map(targetRPM[0],0,40,0,-4380);
  }
  else if (targetRPM[1] == 1){
    cmd_vel[0] = map(targetRPM[0],0,40,0,4380);
  }
  if(targetRPM[3] == 0){
    cmd_vel[1] = map(targetRPM[2],0,40,0,-4380);
  }
  else if (targetRPM[3] == 1){
    cmd_vel[1] = map(targetRPM[2],0,40,0,4380);
  }
  Serial.print(cmd_vel[0]);
  Serial.print(" ");
  Serial.println(cmd_vel[1]);
  if (previous_cmd[0] != cmd_vel[0] || previous_cmd[1] != cmd_vel[1]){
    if (cmd_vel[1] != 0 && cmd_vel[0] != 0){
      roboclaw.SpeedAccelM1(address1,abs(cmd_vel[1]),cmd_vel[1]);
      roboclaw.SpeedAccelM2(address1,abs(cmd_vel[0]),cmd_vel[0]);
      roboclaw.SpeedAccelM1(address2,abs(cmd_vel[0]),cmd_vel[0]);
      roboclaw.SpeedAccelM2(address2,abs(cmd_vel[1]),cmd_vel[1]);
      for(int i = 0; i < sizeof(cmd_vel);i++){
        previous_cmd[i] = cmd_vel[i];
      }
    }
    else if(cmd_vel[1] == 0 && cmd_vel[0] == 0){
      roboclaw.SpeedAccelM1(address1,abs(previous_cmd[1]),cmd_vel[1]);
      roboclaw.SpeedAccelM2(address1,abs(previous_cmd[0]),cmd_vel[0]);
      roboclaw.SpeedAccelM1(address2,abs(previous_cmd[0]),cmd_vel[0]);
      roboclaw.SpeedAccelM2(address2,abs(previous_cmd[1]),cmd_vel[1]);
      for(int i = 0; i < sizeof(cmd_vel);i++){
        previous_cmd[i] = cmd_vel[i];
      }
    }
  }
  /*for(uint8_t i = 0;i<100;i++){
    displayspeed();
    delay(10);
  }*/
  /*roboclaw.SpeedAccelM1(address1,2290,-2290);
  roboclaw.SpeedAccelM2(address1,2290,-2290);
  roboclaw.SpeedAccelM1(address2,2290,-2290);
  roboclaw.SpeedAccelM2(address2,2290,-2290);*/
  /*for(uint8_t i = 0;i<100;i++){
    displayspeed();
    delay(10);
  }*/
  /*roboclaw.SpeedAccelM1(address1,2290,0);
  roboclaw.SpeedAccelM2(address1,2290,0);
  roboclaw.SpeedAccelM1(address2,2290,0);
  roboclaw.SpeedAccelM2(address2,2290,0);*/
}

void requestEvent(){
  Wire.write(targetRPM,4);//Request event funciton must be present in order for I2C master to detect slave.
}

void receiveEvent(int desiredSpeeds) {
  int i = 0;
  Wire.read(); //Get rid of the 0 delimiter coming from the master. The remaining bytes are the valuable values.
  while (Wire.available()) { // loop through all data
    targetRPM[i] = Wire.read(); // receive byte as a byte
    i++;
  }
}
