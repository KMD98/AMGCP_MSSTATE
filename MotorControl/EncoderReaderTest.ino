#include <util/atomic.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>

SoftwareSerial SWSerial(NOT_A_PIN, 8);
Sabertooth ST(128,SWSerial);
// Pins
#define ENCA 2
#define ENCB 3
//globals
long prevT = 0;
int posPrev = 0;
//interrupt variables
volatile int pos_i = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  SWSerial.begin(9600);
  ST.autobaud();
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() { 
  // put your main code here, to run repeatedly:
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
  }
  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  float v1 = velocity1/5120.0*60.0; 
  posPrev = pos;
  prevT = currT;
  ST.motor(1,10);
  Serial.print(v1);Serial.println(" rpm");
  delay(1);
}

void readEncoder(){
  /*int b = digitalRead(ENCB); This does not work due to ENB using digitalread, it needs to be on interrupt because encoder is too fast for this uC.
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }*/
  int increment = 1;
  pos_i = pos_i + increment;
}
