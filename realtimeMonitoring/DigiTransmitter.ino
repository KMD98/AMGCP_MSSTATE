#include "SoftwareSerial.h"

SoftwareSerial XBee(13,15); // 13 is rx and 15 is tx of NODEMCU

void setup()

{

  Serial.begin(19200);
  XBee.begin(19200);

}

void loop()

{
    //data format is !mgcp_coor!drone_coor!mgcp_heading!hotpaneltemp!coldpaneltemp#
    XBee.write("!33.478201,-88.9328992!36.940201,-98.2920392!30.23!90.02!85.02#");
    delay(1000);
}
