/* Creator: Kha Dan
 *  Occupantion: Research Engineer with GRI
 *  Code Purpose: Manual controller that sends logic enable bits and desired rpm commands to powertrain of the AMGCP.
 */
#include<SoftwareSerial.h>
SoftwareSerial xbee(PA10,PA9);
#define vrx PA6
#define vry PA7
#define max_speed 45
#define differential_speed 20 //Always less than max speed and ensure max_speed - differential is not smaller than 20rpm.
#define zeroturn_speed 23
int sw[] = {PB0,PB1,PB2}; // RTK_enable, ON/OFF operation, AUTONOMOUS/RADIO
int* sw_p = sw; 
int commands[] = {0,0,0,0,0,0,0}; //Driver rpm, driver side dir, passenger rpm, passenger dir, RTK_enable, ON/OFF operation, AUTONOMOUS/RADIO
int* comm_pointer = commands;
//Data to send to xbee
char data_sent[] = {'!','0','0','/','0','/','0','0','/','0','/','0','/','0','/','0','#'};
char* data_pointer = data_sent;
void setup() {
  for(int c = 0; c < 3; c++){
    pinMode(*(sw_p + c),INPUT);
  }
  pinMode(vrx, INPUT);
  pinMode(vry,INPUT);
  Serial.begin(19200);
  xbee.begin(19200);
}

void loop() {
  //Read the logic pins
  sw_read();
  // Get the commands and initialize the command array with the values from joysticks
  get_commands();
  //Sends data to Xbee receiver on jetson nano.
  toXbee();
  delay(500);
}

void sw_read(){
  //Read the logic pins
  for(int i = 0; i < 3; i++){
    // add 4 because 4 is the offset in the commands array. Adding 4 let the pointer starts at RTK_enable element
    *(comm_pointer + 4 + i) = digitalRead(*(sw_p+i));
  }
}

void get_commands(){
  int temp[] = {0,0};
  temp[0] = map(analogRead(vrx),0,1023,0,100);
  temp[1] = map(analogRead(vry),0,1023,0,100);
  int temp_rpm;
  int dir;
  int right_speed;
  //Find the rpm values in the x direction
  if(temp[0] <48){ //The joystick is pushed forward and the value decreases as the push increase
    temp_rpm = map(temp[0],0,47,max_speed,21);
    dir = 0; //go forward
  }
  else if(temp[0] > 55){
    temp_rpm = map(temp[0],55,100,21,max_speed); // Joystick is pulled back and value increases as the pull distance increase
    dir = 1; //go backwards
  }
  else{
    dir = 0;
    temp_rpm = 0;
  }
  *(comm_pointer+1) = dir;
  *(comm_pointer+3) = dir;  
  // Alternate the rpm that is appropriate to the yvalue joystick
  if(temp[1] <48 && temp_rpm != 0){ // the right joystick is steered right with decreasing value towards right. The middle value of joy stick is normally 49, calibration will be needed as time goes by.
    right_speed = map(temp[1],0,47,differential_speed,0);
    *(comm_pointer + 2) = temp_rpm - right_speed;
    *(comm_pointer) = temp_rpm;
  }
  else if (temp[1] > 55 && temp_rpm != 0){ //the right joystick is steered left with increasing value towards left
    right_speed = map(temp[1],55,100,0,differential_speed);
    *(comm_pointer) = temp_rpm - right_speed;
    *(comm_pointer + 2) = temp_rpm;
  }
  else if(temp[1] <48 && temp_rpm == 0){ //Activate zero turn towards the right.
    right_speed = map(temp[1],0,47,zeroturn_speed,0);
    *(comm_pointer + 2) = right_speed;
    *(comm_pointer) = right_speed;
    //Flip the direction that is accordance to turning right with zero turn radius.
    *(comm_pointer+1) = 0;
    *(comm_pointer+3) = 1;
  }
  else if(temp[1] > 55 && temp_rpm == 0){ //Activate zero turn towards the left.
    right_speed = map(temp[1],55,100,0,zeroturn_speed);
    *(comm_pointer + 2) = right_speed;
    *(comm_pointer) = right_speed;
    //Flip the direction that is accordance to turning right with zero turn radius.
    *(comm_pointer+1) = 1;
    *(comm_pointer+3) = 0;
  }
  else{
     *(comm_pointer) = temp_rpm;
     *(comm_pointer + 2) = temp_rpm;
  }
}
void toXbee(){
  //Note that xbee likes to receive characters inform of HEX and follows ASCII convention. This function converts commands to a char array and sends it.
  /*for(int i = 0; i < sizeof(commands);i++){ //Printing on COM what data is being sent for debugging purposes
    Serial.print(*(comm_pointer + i));
    delay(1);
  }
  Serial.println();*/
  // Convert contents in comm_pointer from int to char
  convertBytetoChar();
  // Send data out
  for(int c = 0; c < sizeof(data_sent);c++){
    xbee.write(*(data_pointer +c));
    Serial.print(*(data_pointer+c));Serial.print(" ");
    delay(1);
  }
  Serial.println();
}
void convertBytetoChar(){
  //Convert the two rpm values at index 0 and 2 into separate values per index in the data_sent array
  data_sent[2] = (*comm_pointer) % 10 + '0';
  data_sent[1] = ((*comm_pointer)/10) %10 + '0';
  data_sent[7] = (*(comm_pointer + 2)) % 10 + '0';
  data_sent[6] = ((*(comm_pointer + 2))/10) % 10 + '0';
  data_sent[4] = *(comm_pointer+1) + '0';
  data_sent[9] = *(comm_pointer+3) + '0';
  data_sent[11] = *(comm_pointer + 4) + '0';
  data_sent[13] = *(comm_pointer + 5) + '0';
  data_sent[15] = *(comm_pointer + 6) + '0';
}
