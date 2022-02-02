/* Creator: Kha Dan
 * Date: February 1st, 2022
 * Description:The purpose of the code is to use a sabertooth 2x60 driver to drive 4 brushed t74 motors 
 * from NPC with CUI encoders.
 */
class BrushedDCPID{
  private:
    float kp,ki,kd,umax;
    float eintegral, eprev;
  public:
  //Constructor
  BrushedDCPID():kp(1), ki(0), kd(0), umax(255),eprev(0.0),eintegral(0.0){}

  //Call this function to set the parameters or find constant.
  void setParams(float kpIn, float kiIn, float kdIn, float umaxIn){
    kp = kpIn; ki = kiIn; kd = kdIn; umax = umaxIn;
  }
  //Function to computer output pwm signal, u. We pass in duty and dir address and this class will set values by reference
  void evaluate(int rpm, int target, float deltaT, int &duty, int &dir){
    // error
    int e = target - rpm;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
    
    // Set the motor speed and direction. CW in this case. Will need to change to array of dir based on I2C input
    int dir = 1;
    int duty = (int) fabs(u); //We grab the magnitude of u because signs only show direction of voltage. Direction will be determined by high level controller.
    if(u >= umax){
      duty = umax;
    }
    else if ( u <= 0){
      duty = 0;
    }    
    if(target <= 1){ // if target is 1 rpm or 0 rpm then just stop moving
      duty = 0;
    }
    // store previous error
    eprev = e;
  }
};

// Define motor amount and potentiometer pin
#define POTPassenger PA7
#define NMOTORS 4 //Change going from PeeWee to Betty, vice-versa
#define POTDriver PA6

// Declare pins and change with respect to wiring. Note that each index must corresponds.
// For example: enca[0] = PB12 must be the encoder for PID loop that controls PWM[0] = PB0
const int enca[] ={PB12,PB13,PB14,PB15};
const int PWM[] = {PB0,PB1,PB2,PB3}; // O index is driver side (targetRPM[0]), 1 index is passenger side (targetRPM[1]), 2 index is driver side rear(targetRPM[0]), 3 index is passenger side rear (targetRPM[1])


//PID Globals that are not in the class
unsigned long prevTPID = 0;

//RPM calculator global
unsigned long previousT[] = {0,0,0,0};
int distance[] = {0,0,0,0};
int prevpos[] = {0,0,0,0};
long rpm[] = {0,0,0,0};
int targetRPM[] = {0,0};

//interrupt variables
volatile int pos_i[] = {0,0,0,0};

//Everytime interrupt is called. Template will index it with respect to the encoder.
template<int j>
void readEncoder(){
  int increment = 1;
  pos_i[j] = pos_i[j] + increment;
}

//Create PID instance
BrushedDCPID pid[NMOTORS];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for(int i = 0; i < NMOTORS; k++){
    pinMode(enca[i],INPUT);
    pinMode(PWM[i],OUTPUT);
  }
  // Set param for PIDS
  pid[0].setParams(1,0,0,255) 
  pid[1].setParams(1,0,0,255)
  pid[2].setParams(1,0.65,0.07,255)//currently labeled as motor 3
  pid[3].setParams(1,0,0,255)

  pinMode(POT, INPUT);
  
  //Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]),readEncoder<2>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]),readEncoder<3>,RISING);

}

void loop() {
  //Find target
  int pot_valPass = analogRead(POTPassenger);
  int pot_valDriver = analogRead(POTDriver);
  targetRPM[0] = map(pot_valPass,0,1024,0,37); // Our max is 37rpm to be safe.
  targetRPM[1] = map(pot_valDriver,0,1024,0,37)
  //Get rpm
  int pos[] = {0,0,0,0};
  //Determine rpm for all 4 motors and set current to previous for next iteration.
  for (int enc_num = 0; i < sizeof(pos); enc_num++){
    pos[enc_num] = pos_i[enc_num];
    distance[enc_num] = pos[enc_num]-prevpos[enc_num]
    unsigned long currT = micros();
    unsigned long delT = currT - previousT[enc_num];
    rpm[enc_num] = (distance[enc_num]*1000000)/delT * 100000 / 1920 * 60 / 100000;
    prevpos[enc_num] = pos[enc_num];
    previousT[enc_num] = currT
  }  
  // time difference for PID
  unsigned long currTPID = micros();
  float deltaTPID = ((float) (currTPID - prevTPID))/1e6;
  
  // Calculate duty for each motor
  for(int k = 0;k < sizeof(NMOTORS);k++){
    int duty,dir;    
    //duty and dir are passed by reference, they will be used to set the motor speed
    pid[k].evaluate(rpm[k],targetRPM[k%2],deltaTPID,duty,dir);
    // signal the motor
    setMotor(dir,duty,PWM[k]);
    // Print results
  }
  Serial.print(target);
  Serial.print(" ");
  Serial.print(rpm[0]);
  Serial.print(" ");
  Serial.print(rpm[1]);
  Serial.print(" ");
  Serial.print(rpm[2]);
  Serial.print(" ");
  Serial.print(rpm[3]);
  Serial.println();  
  //Set previous time for PID
  prevTPID = currTPID;
  delay(100); // the program runs faster than interrupt slowest time of 50ms. I put a delay of 100ms here to make sure we let interrupt run a couple of times before the next iteration of the program. This prevent losing rpm and time value.
}

void setMotor(int dir, int pwmVal, int pwmPin){
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
    // Or dont turn where 129 will have an input voltage of near 2.5V and Sabertooth stop motor at 2.5V.
    analogWrite(pwmPin,129);   
  }
}
