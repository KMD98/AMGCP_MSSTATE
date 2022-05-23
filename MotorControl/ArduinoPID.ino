
// Example modded by Dan to control both motors on one drive to go forward then backwards. Default baud is 9600

#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <Wire.h>
SoftwareSerial SWSerial(NOT_A_PIN, 8); // RX on no pin (unused), TX on pin 11 (to S1).
Sabertooth ST(128, SWSerial); // Address 128, and use SWSerial as the serial port.
const int buttonPin = 2;     // the number of the pushbutton pin. Please make sure push button is connected to ground or use internal pulldown else the push button will float when let go.
volatile int button_state;
bool keep_going = true;
void setup()
{
  SWSerial.begin(9600);
  ST.autobaud();
  Wire.begin(0x06);
  Wire.onRequest(requestEvent);
  pinMode(buttonPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(buttonPin), pin_ISR, CHANGE);
}

void loop()
{
  int power;
  
  // Ramp from -127 to 127 (full reverse to full forward), waiting 20 ms (1/50th of a second) per value.
  if (button_state == HIGH && keep_going){
    for (power = 1; power <= 32; power ++)
    {
      //Positive means going backwards/CW if look from behind motor. Negative means going CCW/forward on the AMGCP.
      ST.motor(1, power); //pass side.
      delay(20);
      ST.motor(2,-power); // driver side
      delay(20);
    }
    keep_going = false;

  }
  else if (button_state == LOW && !keep_going){
    for (power = 32; power >= 0; power--)
    {
      ST.motor(1, power);
      delay(20);
      ST.motor(2,-power); //Driver side
      delay(20);
    }
    keep_going = true;
  }
  /*// Now go back the way we came.
  for (power = 127; power >= -127; power --)
  {
    ST.motor(1, power);
    delay(20);
    ST.motor(2,power);
    delay(20);
  }*/
}

void pin_ISR(){
  button_state = digitalRead(buttonPin);
}
void requestEvent(){
  
}
