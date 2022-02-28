# Information
  - PID with sabertooth for 4 npc t74 motors
  - Because this is arduino ide programming stm32 uc. It is best to not use class structures to do the programming to avoid bug. The pid class code is deprecated simply because the API could not translate that code into STM environment
  - Note that all I2C code in arduino IDE must have function onRequest in order for high level controller/master to be able to detect the device.
  - Deploy STM32PID.ino on an stm32f4ceu11 uC to have a 4 motor controlled PID. Note that the uC must be connected to a 3.3v to 5V logic converter. We used an adafruit bidirectional logic converter with 4 inputs and 4 outputs.
