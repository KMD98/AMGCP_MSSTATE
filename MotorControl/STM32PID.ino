/*
 * Creator: Kha Dan Research Engineering GRI
 * Description: PID algorithm to control four t74 motor on two sabertooth 2x60 with 4 encoders.
 */
// Pins. Note that ENCAM1 is the feedback signal for PWM1. 
#define ENCAM1 PB7//motor 2
#define ENCAM2 PB12 //motor 1
#define ENCAM3 PB13
#define ENCAM4 PB14
#define PWM1 PB1 //motor 2
#define PWM2 PB0 //motor 1
#define PWM3 PA6
#define PWM4 PB5
#define POT PA7
const int NMOTORS = 4;
//PID Globals
unsigned long prevTPID = 0;
float eintegral = 0;
float eprev = 0;
//RPM calculator global
unsigned long previousT = 0;
int distance[] = {0,0,0,0};
int prevpos[] = {0,0,0,0};
int rpm[] = {0,0,0,0};
int targetRPM[] = {0,0};
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
  pinMode(POT,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCAM1),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCAM2),readEncoder<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCAM3),readEncoder<2>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCAM4),readEncoder<3>,RISING);

}

void loop() {
  int pot_val = analogRead(POT);
  targetRPM[0] = map(pot_val,0,1023,0,43); // Our max is 37rpm to be safe.
  targetRPM[1] = targetRPM[0];
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
  call_PID(rpm[0],deltaTPID,targetRPM[0],1,0.65,0,PWM1,1); // motor 2
  call_PID(rpm[1],deltaTPID,targetRPM[1],1,0.65,0,PWM2,1); //motor 1

  //Set previous time for rpm.
  previousT = currT;
  //Set previous for PID
  prevTPID = currTPID;
  Serial.print(targetRPM[0]);
  Serial.print(" ");
  Serial.print(rpm[0]);
  Serial.print(" ");
  Serial.print(rpm[1]);
  Serial.println();
  delay(100); // the program runs faster than interrupt slowest time of 50ms. I put a delay of 100ms here to make sure we let interrupt run a couple of times before the next iteration of the program. This prevent losing rpm and time value.
}

void call_PID(int rpm, float deltaTPID,int target,float kp,float ki,float kd,int pwmPin,int dir){
  // Compute the control signal u
  float e = target-rpm;
  float dedt = (e - eprev)/deltaTPID;
  eintegral = eintegral + e*deltaTPID;
  eprev = e;
  //Serial.print(kp*e);Serial.print(" ");Serial.print(ki*eintegral);Serial.print(" "); Serial.println(kd*dedt);
  float u = kp*e + ki*eintegral + kd*dedt;
  // Set the motor speed. Direction is CW in this case. Will need to change to array of dir.
  int duty = (int) fabs(u); //We grab the magnitude of u because signs only show direction of voltage. Direction will be determined by high level controller.
  if(u >= 255){
    duty = 255;
  }
  else if ( u <= 0){
    duty = 0;
  }

  if(target <= 1){ // if target is 1 rpm or 0 rpm then just stop moving
    duty = 0;
  }
  setMotor(dir,duty,pwmPin);
  delay(1);  
}
void setMotor(int dir, int pwmVal,int pwmPin){
  if(dir == 1){ 
    // Turn CW
    pwmVal = map(pwmVal,0,255,129,255); //Experimental result with sabertooth and logic converter show 129 to be CW nearly stopped speed
    analogWrite(pwmPin,pwmVal);
  }
  else if(dir == -1){
    // Turn CCW
    pwmVal = map(pwmVal,0,255,129,0); //Experiemental result with sabertooth and logic converter show 129 to be CCW nearly stopped speed
    analogWrite(pwmPin,pwmVal);
  }
  else{
    // Or dont turn where 130 will have an input voltage of near 2.5V and Sabertooth stop motor at 2.5V.
    analogWrite(pwmPin,129);   
  }
}
