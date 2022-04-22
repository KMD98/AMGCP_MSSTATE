/*
 * Creator: Kha Dan Research Engineering GRI
 * Description: PID algorithm to control four t74 motor on two sabertooth 2x60 with 4 encoders.
 */
#include <Wire.h>
// Pins. Note that ENCAM1 is the feedback signal for PWM1. MAKE SURE THAT EACH OUTPUT AND INPUT CORRESPONDS TO THE RIGHT MOTOR WHEN WIRING 
#define ENCAM1 PB7//motor 1 driver side rear
#define ENCAM2 PB12 //motor 2 passenger side front
#define ENCAM3 PB13 //motor 3 driver side front
#define ENCAM4 PB14 //motor 4 passender side rear
int PWM[] = {PA6,PB0,PB1,PA5}; //motor 1,motor2,motor3,motor4. MAKE SURE MOTORS ON ONE DRIVER ARE ON ON OPPOSITE SIDES.
TwoWire Wire2(PB9,PB8);
const int NMOTORS = 4;
//PID Globals
unsigned long prevTPID = 0;
float eintegral[] = {0,0,0,0};
float eprev[] = {0,0,0,0};
float kp[] = {1.0,1.0,1.0,1.0};
float ki[] = {0.65,0.65,0.65,0.65};
float kd[] = {0.0,0.0,0.0,0.0};
int no_movement[] = {128,126};
//RPM calculator global
unsigned long previousT = 0;
int distance[] = {0,0,0,0};
int prevpos[] = {0,0,0,0};
int rpm[] = {0,0,0,0};
//I2C global. Set by high level computer
byte targetRPM[] = {0,0,0,0}; //{passenger side,passenger direction 1 or 0, driver, driver direction} where 1 is CW and 0 is CCW
byte requestData[] = {0,0,0,0,0,0,0,0}; //{target pass, target pass dir, target driver, target driver dir, m1 passenger, m2 driver,m3 passenger, m4 driver
//interrupt variables
volatile int pos_i[] = {0,0,0,0};

template<int j>
void readEncoder(){
  int increment = 1;
  pos_i[j] = pos_i[j] + increment;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire2.begin(0x6);
  pinMode(ENCAM1, INPUT);
  pinMode(ENCAM2, INPUT);
  pinMode(ENCAM3, INPUT);
  pinMode(ENCAM4, INPUT);
  Wire2.onReceive(receiveEvent);
  Wire2.onRequest(requestEvent);//Must be present so master can detect slave. Else i2c will bug.
  attachInterrupt(digitalPinToInterrupt(ENCAM1),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCAM2),readEncoder<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCAM3),readEncoder<2>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCAM4),readEncoder<3>,RISING);

}


void loop() {
  //Get rpm  
  unsigned long currT = micros();
  unsigned long delT = currT - previousT;
  int pos[] = {0,0,0,0};
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
  call_PID(rpm,deltaTPID,targetRPM,kp,ki,kd,PWM);
  
  //Set previous time for rpm.
  previousT = currT;
  //Set previous for PID
  prevTPID = currTPID;
  Serial.print(targetRPM[0]);//driver side
  Serial.print(" ");
  Serial.print(targetRPM[2]);//passenger side
  Serial.print(" ");
  Serial.print(rpm[0]);
  Serial.print(" ");
  Serial.print(rpm[1]);
  Serial.print(" ");
  Serial.print(rpm[2]);
  Serial.print(" ");
  Serial.print(rpm[3]);
  Serial.println();
  delay(100); // the program runs faster than interrupt slowest time of 50ms. I put a delay of 100ms here to make sure we let interrupt run a couple of times before the next iteration of the program. This prevent losing rpm and time value.
}

void call_PID(int rpm[], float deltaTPID,byte goal[],float kp[],float ki[],float kd[],int pwmPin[]){
  // Compute the control signal u
  float e[] = {0,0,0,0};
  float dedt[] = {0,0,0,0};
  int duty[] = {0,0,0,0};
  int dir[] = {0,0,0,0};
  int target;
  for(int i = 0; i < 4; i++){
    if(i%2 == 0){
      target = goal[0]; //set rpm target and direction for driver side
      dir[i] = goal[1];
    }
    else{
      target = goal[2];//set rpm target and direction for passenger side
      dir[i] = goal[3];
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
    else if ( u <= 0){
      duty[i] = 0;
    }
  
    if(target <= 3){ // if target is 8 or less rpm or 0 rpm then just stop moving
      duty[i] = 0;
    }
  }
  //Send pwm to each motor
  for(int c = 0; c<4; c++){
    int temp;
    if (c == 1 || c == 2){
      temp = no_movement[0];
    }
    else{
      temp = no_movement[1];
    }
    setMotor(dir[c],duty[c],pwmPin[c],temp);
    delay(1);
  }
}
void setMotor(int dir, int pwmVal,int pwmPin, int halt){
  if(dir == 1 && pwmVal != 0){ 
    // Turn CW if looking at motor from behind.
    pwmVal = map(pwmVal,0,255,halt,255); //Experimental result with sabertooth and logic converter show 129 to be CW nearly stopped speed
    analogWrite(pwmPin,pwmVal);
  }
  else if(dir == 0 && pwmVal != 0){
    // Turn CCW
    pwmVal = map(pwmVal,0,255,halt,0); //Experiemental result with sabertooth and logic converter show 129 to be CCW nearly stopped speed
    analogWrite(pwmPin,pwmVal);
  }
  else{
    // Or dont turn where 129 will have an input voltage of near 2.5V and Sabertooth stop motor at 2.5V for the motor driver that drives motor 4 and motor 1. Motor driver 2 and 3 on another sabertooth driver requires 128 as the stopping speed.
    analogWrite(pwmPin,halt);   
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
  Wire2.write(requestData,8);//Request event funciton must be present in order for I2C master to detect slave.
}

void receiveEvent(int desiredSpeeds) {
  int i = 0;
  Wire2.read(); //Get rid of the 0 delimiter coming from the master. The remaining bytes are the valuable values.
  while (Wire2.available()) { // loop through all data
    targetRPM[i] = Wire2.read(); // receive byte as a byte
    i++;
  }
}
