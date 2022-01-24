#include <util/atomic.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>

// Pins
#define ENCA 2
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
  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
}

void loop() {

  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;
  // Convert count/s to RPM
  float v1 = velocity1/960.0*60.0; // 600 is per this example. Ours is 1 revolution of motor shaft per 48 pulses or count and 1/20 revolution at geared shaft per 1 revolution of motor shaft. That gives 5120 instead of 600.

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;


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
  if(pwr > 35){
    pwr = 35;
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
  int increment = 1;
  pos_i = pos_i + increment;

}
