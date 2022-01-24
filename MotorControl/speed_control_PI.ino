#include <util/atomic.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>

// Pins
#define ENCA 3
#define ENCB 4
#define PWM 8
SoftwareSerial SWSerial(NOT_A_PIN, PWM);//TX on pin 8 to Sabertooth
Sabertooth ST(128,SWSerial);
// globals
long prevT = 0;
int posPrev = 0;
// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

void setup() {
  Serial.begin(115200);
  SWSerial.begin(9600);
  ST.autobaud();
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
}

void loop() {

  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert count/s to RPM
  float v1 = velocity1/5120.0*60.0; // 600 is per this example. Ours is 1 revolution of motor shaft per 256 pulses or count and 1/20 revolution at geared shaft per 1 revolution of motor shaft. That gives 5120 instead of 600.
  float v2 = velocity2/5120.0*60.0; // The units stochoimetry is pulses/second * 1 revoMotorShaft/256 Pulses * 1 revoGearedShaft/20 revoMotorShaft = revoGearedShaft/second * 60 second/min = rpm

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
  v2Prev = v2;

  // Set a target
  float vt = 50; // Ours is 50rpm, 

  // Compute the control signal u
  float kp = 5;
  float ki = 10;
  float e = vt-v1Filt;
  eintegral = eintegral + e*deltaT;
  
  float u = kp*e + ki*eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u); //We grab the magnitude of u because signs only show direction of voltage.
  if(pwr > 64){
    pwr = 64;
  }
  setMotor(dir,pwr);

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.println();
  delay(1);
}

void setMotor(int dir, int pwmVal){
  ST.motor(1,pwmVal);// set motor speed
  if(dir == 1){ 
    // Turn CW
    ST.motor(1,pwmVal);
  }
  else if(dir == -1){
    // Turn CCW
    ST.motor(1,-pwmVal);
  }
  else{
    // Or dont turn
    ST.motor(1,0);    
  }
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;

  // Compute velocity with method 2. This method is really bad because if there is no interrupt...we cannot calculate speed. Do not use method 2.
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}
