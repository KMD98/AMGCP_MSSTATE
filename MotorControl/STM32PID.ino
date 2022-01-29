
// Pins
#define ENCAM1 PB12
#define ENCAM2 PB13
#define ENCAM3 PB14
#define ENCAM4 PB15
#define PWM PB1
//globals
float prev_delT[] = {0.0,0.0,0.0,0.0};
int samedelT_count[]={0,0,0,0};
//PID Globals
long prevTPID = 0;
float eintegral = 0;
float eprev = 0;
//interrupt variables
volatile unsigned int pos_i[] = {0,0,0,0};
volatile long prevT[] = {0,0,0,0}; 
volatile float delT[] = {0,0,0,0};
volatile float rpm[] = {0,0,0,0};


template<int j>
void readEncoder(){
  long currT = micros();
  int increment = 1;
  pos_i[j] = pos_i[j] + increment;
  delT[j] = (float)(currT - prevT[j]);
  rpm[j] = 1000000.0/delT[j] * 100000/1920.0 * 60.0 / 100000.0;//Had to multiply result by 10000 to prevent rounding midway. The result would be in rev/s and it will be rev/s * 60s/min to get rpm. Dont use this because it would randomly round to 0
  prevT[j] = currT;
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
  //Making sure that microcontroller know speed is 0 when motor stops
  if(delT[0] == prev_delT[0]){
    samedelT_count[0]++;
  }
  if(samedelT_count[0] > 3){
    delT[0] = 0;
    rpm[0] = 0;
    samedelT_count[0] = 0;
  }
  // time difference for PID
  long currTPID = micros();
  float deltaTPID = ((float) (currTPID - prevTPID))/( 1.0e6 );
  prevTPID = currTPID;
  prev_delT[0] = delT[0];
  
  call_PID(rpm[0],deltaTPID);

  
}

void call_PID(float rpm, float deltaTPID){
  // Set a target
  float target = 37.0; // Ours is 37rpm
  // Compute the control signal u
  float kp = 1;
  float ki = 0;
  float kd = 0;
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
