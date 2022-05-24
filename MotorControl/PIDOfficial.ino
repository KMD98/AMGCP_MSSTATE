/*
 * Creator: Kha Dan Research Engineering GRI
 * Description: PID algorithm to control four t74 motor on two sabertooth 2x60 with 4 encoders.
 */
#include <SoftwareSerial.h>
#include <Sabertooth.h>
SoftwareSerial SWSerial(NOT_A_PIN, 8); // RX on no pin (unused), TX on pin 11 (to S1).
Sabertooth ST(128, SWSerial); // Address 128, and use SWSerial as the serial port.
#include <Wire.h>
// Pins. Note that ENCAM1 is the feedback signal for PWM1. MAKE SURE THAT EACH OUTPUT AND INPUT CORRESPONDS TO THE RIGHT MOTOR WHEN WIRING 
#define ENCAM1 2 //motor 3 driver side front
#define ENCAM2 3 //motor 2 passenger side front
const int NMOTORS = 2;
int pwmPins[] = {2,1};
//PID Globals
unsigned long prevTPID = 0;
float eintegral[] = {0,0};
float eprev[] = {0,0};
float kp[] = {1.0,1.0};
float ki[] = {0.55,0.55};
float kd[] = {0.0,0.0};
//RPM calculator global
unsigned long previousT = 0;
int distance[] = {0,0};
int prevpos[] = {0,0};
int rpm[] = {0,0}; //0 element is passener side, 1 element is driver side.
//I2C global. Set by high level computer
byte targetRPM[] = {0,0,0,0}; //{driver side,driver direction 1(backwards) or 0 (forward), passenger, passenger direction} where 1 is CW and 0 is CCW
byte requestData[] = {0,0,0,0,0,0}; //{target pass, target pass dir, target driver, target driver dir, m1 passenger, m2 driver,m3 passenger, m4 driver
//interrupt variables
volatile int pos_i[] = {0,0};

bool testing = true;

template<int j>
void readEncoder(){
  int increment = 1;
  pos_i[j] = pos_i[j] + increment;
}

void setup() {
  // put your setup code here, to run once:
  SWSerial.begin(9600);
  ST.autobaud();
  Serial.begin(9600);
  Wire.begin(0x6);
  pinMode(ENCAM1, INPUT);
  pinMode(ENCAM2, INPUT);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);//Must be present so master can detect slave. Else i2c will bug.
  attachInterrupt(digitalPinToInterrupt(ENCAM1),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCAM2),readEncoder<1>,RISING);
}


void loop() {
  //Get rpm  
  unsigned long currT = micros();
  unsigned long delT = currT - previousT;
  int pos[] = {0,0};
  for(int k = 0; k < NMOTORS;k++){
    pos[k] = pos_i[k];
    distance[k] = pos[k] - prevpos[k];
    rpm[k] = (distance[k]*1000000)/delT * 100000 / 1920 * 60 / 100000;
    prevpos[k] = pos[k];
  }
  // time difference for PID
  unsigned long currTPID = micros();
  float deltaTPID = ((float) (currTPID - prevTPID))/1e6;
  //Execute PID and motor control
  call_PID(rpm,deltaTPID,targetRPM,kp,ki,kd);
  /*if(targetRPM[0] == 30 && testing){
    for(int i = 0; i < 28; i++){
      ST.motor(1,i);
      delay(20);
      ST.motor(2,i);
      delay(20);
    }
    testing = false;
  }
  else if (targetRPM[0] == 0 && !testing){
    for(int i = 27;i > 0; i--){
      ST.motor(1,i);
      delay(20);
      ST.motor(2,i);
      delay(20);
    }
    testing = true;
  }*/
  //Set previous time for rpm.
  previousT = currT;
  //Set previous for PID
  prevTPID = currTPID;
  Serial.print(targetRPM[0]); // Driver side
  Serial.print(" ");
  Serial.print(targetRPM[2]); // Passnger side
  Serial.print(" ");
  Serial.print(rpm[0]); // driver side
  Serial.print(" ");
  Serial.println(rpm[1]); // passenger side
  delay(100); // the program runs faster than interrupt slowest time of 50ms. I put a delay of 100ms here to make sure we let interrupt run a couple of times before the next iteration of the program. This prevent losing rpm and time value.
}

void call_PID(int rpm[], float deltaTPID,byte goal[],float kp[],float ki[],float kd[]){
  // Compute the control signal u
  float e[] = {0,0};
  float dedt[] = {0,0};
  int duty[] = {0,0};
  int dir[] = {0,0};
  int target;
  int pin = 0;
  for(int i = 0; i < 2; i++){
    if(i == 0){
      target = goal[0]; //set rpm target and direction for driver side
      dir[i] = goal[1];
      pin = pwmPins[i];
    }
    else{
      target = goal[2];//set rpm target and direction for passenger side
      dir[i] = goal[3];
      pin = pwmPins[i];
    }
    e[i] = target - rpm[i];
    dedt[i] = (e[i] - eprev[i])/deltaTPID;
    eintegral[i] = eintegral[i] + e[i]*deltaTPID;
    eprev[i] = e[i];
    //Serial.print(kp*e);Serial.print(" ");Serial.print(ki*eintegral);Serial.print(" "); Serial.println(kd*dedt);
    float u = kp[i]*e[i] + ki[i]*eintegral[i] + kd[i]*dedt[i];
    // Set the motor speed. Direction is CW in this case. Will need to change to array of dir.
    duty[i] = (int) fabs(u); //We grab the magnitude of u because signs only show direction of voltage. Direction will be determined by high level controller.
    if(u >= 255){
      duty[i] = 255;
    }
    else if ( u < 0){
      if (dir[i] == 1){
        dir[i] = 0
        }
      else if (dir[i] == 0){
        dir[i] = 1;
      }
    }
    if(target <= 3 && target >= 0){ // if target is 3 or less rpm or 0 rpm then just stop moving because our resolution is about 3 rpm
      duty[i] = 0;
    }
    setMotor(dir[i],duty[i],pin);
  }
}
void setMotor(int dir, int pwmVal,int pwmPin){
  // Map the pid value to appropriate motor driver serial packet value
  pwmVal = map(pwmVal,0,255,0,128);
  if(dir == 1){ 
    // Go backwards need to pass a positive value because of the motor wirings
    ST.motor(pwmPin,pwmVal);
  }  
  else if (dir == 0){
    // Go forward need to pass a negative value because of the motor wirings
    ST.motor(pwmPin,-pwmVal);
  }
  else{
    // Stop the motors
    ST.motor(pwmPin, 0);
  }
}

void requestEvent(){
  //Request data returns data in the following format [tarDriver,dirDriver,tarPass,dirPass,m1speed,m2speed,m3speed,m4speed]
  for(int i = 0; i < sizeof(targetRPM); i++){
    requestData[i] = targetRPM[i]; 
  }
  for(int i = 0; i < sizeof(rpm); i++){
    requestData[i+4] = byte(rpm[i]);
  }
  Wire.write(requestData,6);//Request event funciton must be present in order for I2C master to detect slave.
}

void receiveEvent(int desiredSpeeds) {
  int i = 0;
  Wire.read(); //Get rid of the 0 delimiter coming from the master. The remaining bytes are the valuable values.
  while (Wire.available()) { // loop through all data
    targetRPM[i] = Wire.read(); // receive byte as a byte
    i++;
  }
}
