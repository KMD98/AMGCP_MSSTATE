
// Pins
#define ENCAM1 PB12
#define ENCAM2 PB13
#define ENCAM3 PB14
#define ENCAM4 PB15
#define PWM PB1
//globals
float prev_delT[] = {0.0,0.0,0.0,0.0};
int samedelT_count[]={0,0,0,0};
float delTFilt = {0,0,0,0};
//interrupt variables
volatile unsigned int pos_i[] = {0,0,0,0};
volatile long prevT[] = {0,0,0,0}; 
volatile float delT[] = {0,0,0,0};


template<int j>
void readEncoder(){
  long currT = micros();
  int increment = 1;
  pos_i[j] = pos_i[j] + increment;
  delT[j] = (float)(currT - prevT[j]);
  //rpm[j] = 100000000000.0/delT/1920.0 * 60.0 / 100000.0;//Had to multiply result by 1000000 to cancel out the us and prevent rounding. The result would be in rev/s and it will be rev/s * 60s/min to get rpm. Dont use this because it would randomly round to 0
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
  if(String(delT[0]) == String(prev_delT[0])){
    samedelT_count[0]++;
  }
  if(samedelT_count[0] > 3){
    delT[0] = 0;
    samedelT_count[0] = 0;
  }
  // time difference for PID
  long currTPID = micros();
  float deltaTPID = ((float) (currTPID - prevTPID))/( 1.0e6 );
  prevTPID = currTPID;
  //Serial.print(delT[0]);Serial.println(" us/pulse with 1rev/96pulses and 20:1 GR");
  //Low-pass filter (25Hz cutoff)
  delTFilt[0] = 0.854*delTFilt[0] + 0.0728*delT[0] + 0.0728*prev_delT[0];
  prev_delT[0] = delT[0];
  call_PID(delTFilt[0],deltaTPID);

  
}

void call_PID(float delTFilt, float deltaTPID){
  // Set a target
  float targerT = 40; // Ours is 40rpm, 

  // Compute the control signal u
  float kp = 1;
  float ki = 0;
  float kd = 0;
  float e = tagetT-delTFilt;
  float dedt = (e - eprev)/deltaTPID;
  eintegral = eintegral + e*deltaTPID;
  eprev = e;
  //Serial.print(kp*e);Serial.print(" ");Serial.print(ki*eintegral);Serial.print(" "); Serial.println(kd*dedt);
  float u = kp*e + ki*eintegral + kd*dedt;
  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u); //We grab the magnitude of u because signs only show direction of voltage.
  if(pwr > 200){
    pwr = 200;
  }
  setMotor(dir,pwr);

  Serial.print(targetT);
  Serial.print(" ");
  Serial.print(delTFilt);
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
    ST.motor(1,);
  }
  else{
    // Or dont turn
    ST.motor(1,0);    
  }
}
