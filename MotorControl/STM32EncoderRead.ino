
// Pins
#define ENCAM1 PB12
#define ENCAM2 PB13
#define ENCAM3 PB14
#define ENCAM4 PB15
#define PWM PB1
//globals
float prev_delT[] = {0.0,0.0,0.0,0.0};
int samedelT_count[]={0,0,0,0};
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
  analogWrite(PWM,100); //Going CCW. Our encoder count in this sytem is always overshooting so our actual speed with PID will be a bit slower.
  delay(5);
  if(String(delT[0]) == String(prev_delT[0])){
    samedelT_count[0]++;
  }
  if(samedelT_count[0] > 3){
    delT[0] = 0;
    samedelT_count[0] = 0;
  }
  Serial.print(delT[0]);Serial.println(" us/pulse with 1rev/96pulses and 20:1 GR");
  prev_delT[0] = delT[0];
  delay(2);
}
