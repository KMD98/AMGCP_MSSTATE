# Information
  - deploy these configuration (Kha Dan Modded recommended) via u-center app on appropriate gps (3 gps in total)
  - Note that these files are 1Hz. Please change RATE setting to 200ms (5Hz)
  - Please disable REPOLNED message on RTK2Blite and POSLLH message on RTK2Blite on UART1.
  - Pleas make sure RTCM messages are also enabled on UART1 on rtk2blite (the moving base).
  - LEAVE RTK2BLITE UART2 ALONE BECAUSE THAT IS THE RTCM RECEIVER FROM BASE STATION
  - Note that RTK2B(heading) UART1 export both NAV-REPOSLNED and NAV-PVT (make sure this is true with your config).
  - WE WANT TO SIMULTANEOUSLY GRAB NAV-PVT AND REPOLSNED (gives heading/bearing), THIS IS WHY WE HAVE TWO BLACK PILL TO DO PARALLEL PROCESSING FROM THE ROVER RTK2B. WE SIMPLY PARALLEL THE SERIAL SIGNAL FROM UART 1 AND FEED TO TWO uC
