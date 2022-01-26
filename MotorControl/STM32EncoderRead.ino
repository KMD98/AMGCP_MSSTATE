
// Pins
#define ENCA PB12
#define PWM PB1
//globals
long prevT = 0;
int posPrev = 0;
//interrupt variables
volatile int pos_i = 0;
void readEncoder(){
  int increment = 1;
  pos_i = pos_i + increment;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() { 
  // put your main code here, to run repeatedly:
  int pos = 0;
  // Set incremented values to local variable
  detachInterrupt(digitalPinToInterrupt(ENCA));
  pos = pos_i; //to test overflow, reset pos_i here to see what happens
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  float v1 = velocity1/1920.0*60.0; 
  posPrev = pos;
  prevT = currT;
  int i = 0;
  /*for(i=150;i<=190; i++){
    analogWrite(PWM,i);
    Serial.print(i);
    delay(5000);
  }*/
  analogWrite(PWM,85); //Going CCW. Our encoder count in this sytem is always overshooting so our actual speed with PID will be a bit slower.
  Serial.print(v1);Serial.println(" rpm");
  delay(1);
}
