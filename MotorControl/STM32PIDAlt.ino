
// Pins
#define ENCAM1 PB12
#define ENCAM2 PB13
#define ENCAM3 PB14
#define ENCAM4 PB15
#define PWM PB1
//PID Globals
long prevTPID = 0;
float eintegral = 0;
float eprev = 0;
//RPM calculator global
unsigned long previousT = 0;
int distance[] = {0,0,0,0};
int prevpos[] = {0,0,0,0};
unsigned long rpm[] = {0,0,0,0};
//interrupt variables
volatile int pos_i[] = {0,0,0,0};



template<int j>
void readEncoder(){
  int increment = 1;
  pos_i[j] = pos_i[j] + increment;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCAM1, INPUT);
  pinMode(ENCAM2, INPUT);
  pinMode(ENCAM3, INPUT);
  pinMode(ENCAM4, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCAM1),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCAM2),readEncoder<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCAM3),readEncoder<2>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCAM4),readEncoder<3>,RISING);

}

void loop() {
  //analogWrite(PWM,160); 60 rpm no load
  //Get rpm
  int pos[] = {0,0,0,0};
  pos[0] = pos_i[0];
  distance[0] = pos[0] - prevpos[0];
  unsigned long currT = micros();
  unsigned long delT = currT - previousT;
  rpm[0] = (distance[0]*1000000)/delT * 100000 / 1920 * 60 / 100000;
  // time difference for PID
  long currTPID = micros();
  float deltaTPID = ((float) (currTPID - prevTPID))/1e6;
  call_PID(rpm[0],deltaTPID);
  //Set previous for rpm. We do this last because we want to get into PID as fast as possible and previous initialization does not have to happen til right before the next iteration.
  previousT = currT;
  prevpos[0] = pos[0];
  //Set previous for PID
  prevTPID = currTPID;
  delay(100); // the program runs faster than interrupt slowest time of 50ms. I put a delay of 100ms here to make sure we let interrupt run a couple of times before the next iteration of the program. This prevent losing rpm and time value.
}

void call_PID(float rpm, float deltaTPID){
  // Set a target
  unsigned long target = 37; // Ours is 37rpm
  // Compute the control signal u
  float kp = 0.5;
  float ki = 0.07;
  float kd = 0.02;
  float e = target-rpm;
  float dedt = (e - eprev)/deltaTPID;
  eintegral = eintegral + e*deltaTPID;
  eprev = e;
  //Serial.print(kp*e);Serial.print(" ");Serial.print(ki*eintegral);Serial.print(" "); Serial.println(kd*dedt);
  float u = kp*e + ki*eintegral + kd*dedt;
  // Set the motor speed and direction. CW in this case. Will need to change to array of dir.
  int dir = 1;
  int duty = (int) fabs(u); //We grab the magnitude of u because signs only show direction of voltage. Direction will be determined by high level controller.
  if(u >= 255){
    duty = 255;
  }
  else if ( u <= 0){
    duty = 0;
  }
  setMotor(dir,duty);

  Serial.print(target);
  Serial.print(" ");
  Serial.print(rpm);
  Serial.println();
  delay(1);  
}
void setMotor(int dir, int pwmVal){
  if(dir == 1){ 
    // Turn CW
    pwmVal = map(pwmVal,0,255,132,255); //Experimental result with sabertooth and logic converter show 132 to be CW nearly stopped speed
    analogWrite(PWM,pwmVal);
  }
  else if(dir == -1){
    // Turn CCW
    pwmVal = map(pwmVal,0,255,127,0); //Experiemental result with sabertooth and logic converter show 127 to be CCW nearly stopped speed
    analogWrite(PWM,pwmVal);
  }
  else{
    // Or dont turn where 130 will have an input voltage of near 2.5V and Sabertooth stop motor at 2.5V.
    analogWrite(PWM,130);   
  }
}
