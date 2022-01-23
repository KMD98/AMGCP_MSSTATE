#include <util/atomic.h> 
#include <Wire.h> //Wire is the I2C library :D. It is the little things like this is why we use arduino to prototype before implementing on ARM controllers.
/* Note that this code passes the number of pulses via I2C bus. To integrate this with our motor,
 * we have to find the number of pulses per rev from our motor design by finding the coil span of the hall sensors
 and the total number of coils in the stator. If the seller does not have this information, we'll have to find out our selves
 by creating a code that rotate the motor by exactly rev and count the pulses within that 1 rev. Repeat trial as many times
 as possible to confirm the number of pulses per rev*/

 /*Jetson nano end of I2C still need to be developed. This code is able to receive and send back its position. Jetson nano code will have to be in c++ for speed
 I plan to device an evaluation test using waveform generator for this code soon.*/
 
/*------------ CLASS ------------*/
class BLDCpid{
  // Only functions or statements within a class can access
  private:
    float kp, kd, ki, umax;
    float eprev, eintegral;
    
  public: //The following code is public, which means anywhere can access the variables.
    // Default initialization list
    BLDCpid() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
    
    // Set the parameters
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    // Evaluate the signal
    void evalu(int value, int target, float deltaT,int &pwr, int &dir){
        
      // error
      int e = target - value;
      
      float dedt = (e-eprev)/(deltaT);
      eintegral = eintegral + e*deltaT;
      float u = kp*e + kd*dedt + ki*eintegral;
    
      // motor power
      pwr = (int) fabs(u);
      if( pwr > umax ){
        pwr = umax;
      }
           
      // motor direction
      dir = 1;
      if(u<0){
        dir = -1;
      }
            
      // store previous error
      eprev = e;
    }
    
};

/*------------ GLOBALS AND DEFINITIONS ------------*/
// Define the motors
#define NMOTORS 2
#define M0 0
#define M1 1
// The motor specs says there are only two encoders (hall sensor) per motor so we will only have a and b per motor for now. If they tell me there is three then we will include an encc but that wont change much since
// we only need one encoder to analyze speed and pos. Note that these pin choies are random. I would never use pin 0 or 1 because that's UART.
const int enca[] = {0,1};
const int encb[]= {4,5};
const int pwm[] = {9,13};
const int in1[] = {8,11};
const int in2[] = {10,12};

// Global variables
long prevT = 0;
int posPrev[] = {0,0};

// positions
volatile int posi[] = {0,0};
volatile long pos[] = {0,0};
volatile long target[] = {0,0};

// PID classes
BLDCpid pid[NMOTORS];


/*------------ FUNCTIONS ------------*/
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

/* We use a template here because our encoders are on interrupt for two motors. We dont want to add if statements
in the function to differentiate because that would increase compile and run time, produce repetitive code,etc.Instead, 
we create a template that will create different variant of the function for encoder 0 and enconder 1 as they are called. 
Each variant is of type int.*/
template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}

void sendLong(long value){
  for(int k=0; k<4; k++){
    //Please refer to I2C on arduino if desired to understand the bit to bit operation.
    byte out = (value >> 8*(3-k)) & 0xFF;
    Wire.write(out);
  }
}

long receiveLong(){
  long outValue;
  for(int k=0; k<4; k++){
    byte nextByte = Wire.read();
    //shift by 8bits 4 times to go through the entire 32 bits
    outValue = (outValue << 8) | nextByte;
  }
  return outValue;
}

void requestEvent(){
  sendLong(pos[0]);
  sendLong(pos[1]);
}

void receiveEvent(int howMany){
  target[0] = receiveLong();
  target[1] = receiveLong();
}

void setup() {
  Wire.begin(1); // join I2C
  Wire.onRequest(requestEvent); 
  Wire.onReceive(receiveEvent);
  
  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    // Change the constants as we need to but use default for now because we dont have the motor and system
    pid[k].setParams(1,0,0,255);
  }
  //We want to use enca as the trigger and encoderb as the counter because b always lag behind a by some phase that is dependent on their placement. When a rises, we call template interrupt and see how many pulses b will generate.
  // Again, we dont want to use a because we can miss out on some pulse count if the rpm is fast enough.
  attachInterrupt(digitalPinToInterrupt(enca[M0]),readEncoder<M0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[M1]),readEncoder<M1>,RISING);
}

/*------------ LOOP ------------*/
void loop() {

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT; // Store the previous time value

  // Read the position in an atomic block to avoid a potential misread i.e. disable interrupts and anything that can interfere.
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }

  // Loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir); // compute the position
    setMotor(dir,pwr,pwm[k],in1[k],in2[k]); // signal the motor
  }

  //Loop to print pos but remember that pos is in pulses
  for(int i = 0; i < NMOTORS; i++){
    Serial.print(pos[i]);
    Serial.print(" pulses ");
  }
  /*
   * If pulses to meter is desired, we must find the number of pulses per rev for that motor then we can do something like this in a template function
   * pulseTometerpos[i] = pos[i] / (pulse/rev) * (m/rev)
   */
   
  //Print the new line for next iteration of position
  Serial.println();
}
