
// Pins
#define ENCA PB0
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
  pos = pos_i;
  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  float v1 = velocity1/960.0*60.0; 
  posPrev = pos;
  prevT = currT;
  analogWrite(PWM,150);
  delay(3000);
  analogWrite(PWM,180);
  delay(3000);
  analogWrite(PWM,200);
  delay(3000);
  Serial.print(v1);Serial.println(" rpm");
  delay(1);
}
