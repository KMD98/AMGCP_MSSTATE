/*
 * Creator: Kha Dan Research Engineering GRI
 * Description: direction reader for PID controller (arduino promini). 
 * The stm32 reads encoder inputs at 5v logic, calculates whether if signal A or signal B leads and pull up a pin that is connected to the promini to indicate the direction for each motor.
 * The promini is too slow for this time sensitive task, so that is why the code is on an stm32.
 */
// Pins. Note that ENCAM1 is the feedback signal for PWM1. MAKE SURE THAT EACH OUTPUT AND INPUT CORRESPONDS TO THE RIGHT MOTOR WHEN WIRING 
#define ENCAM1 PB7//motor 1 driver side rear A
#define ENCAM2 PB12 //motor 2 passenger side front A
#define ENCBM1 PB13 //motor 1 driver side rear B
#define ENCBM2 PB14 //motor 2 passenger side front B
#define driver_dir PB0 // Driver side direction
#define pass_dir PB1 // PASSSENGER SIDE DIRECTION
const int NMOTORS = 2;
//RPM calculator global
unsigned long previousT = 0;
int distance[] = {0,0,0,0};
int prevpos[] = {0,0,0,0};
int rpm[] = {0,0,0,0};
//I2C global. Set by high level computer
byte targetRPM[] = {0,0,0,0}; //{driver side,driver direction 1(backwards) or 0 (forward), passenger, passenger direction} where 1 is CW and 0 is CCW
byte requestData[] = {0,0,0,0,0,0,0,0}; //{target pass, target pass dir, target driver, target driver dir, m1 passenger, m2 driver,m3 passenger, m4 driver
//interrupt variables
volatile int motor_dir[] = {0,0,0,0};
volatile int pos_i[] = {0,0,0,0};

template<int j>
void readEncoder(){
  if(j == 0){
    int temp = digitalRead(ENCBM1);  
    if (temp > 0){
      motor_dir[0] = 0;  
    }
    else{
      motor_dir[0] = 1;
    }
    
  }
  else if (j == 1){
    int temp = digitalRead(ENCBM2);
    if (temp > 0){
      motor_dir[1] = 1;
    }
    else{
      motor_dir[1] = 0;
    }
  }
  int increment = 1;
  pos_i[j] = pos_i[j] + increment;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ENCAM1, INPUT);
  pinMode(ENCAM2, INPUT);
  pinMode(ENCBM1, INPUT);
  pinMode(ENCBM2, INPUT);
  pinMode(driver_dir, OUTPUT);
  pinMode(pass_dir, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCAM1),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCAM2),readEncoder<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCBM1),readEncoder<2>,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCBM2),readEncoder<3>,RISING);

}


void loop() {
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
  //Set previous time for rpm.
  previousT = currT;
  // Write direction to pro mini
  digitalWrite(driver_dir, motor_dir[0]);
  digitalWrite(pass_dir, motor_dir[1]);
  //Print debugger
  Serial.print(targetRPM[0]);//driver side
  Serial.print(" ");
  Serial.print(targetRPM[2]);//passenger side
  Serial.print(" ");
  Serial.print(rpm[0]);
  Serial.print(" ");
  Serial.print(rpm[1]);
  Serial.print(" ");
  Serial.print(motor_dir[0]);
  Serial.print(" ");
  Serial.println(motor_dir[1]);
  delay(100); // the program runs faster than interrupt slowest time of 50ms. I put a delay of 100ms here to make sure we let interrupt run a couple of times before the next iteration of the program. This prevent losing rpm and time value.
}
