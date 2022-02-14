PID with sabertooth for 4 npc t74 motors
Because this is arduino ide programming stm32 uc. It is best to not use class structures to do the programming to avoid bug. The pid class code is deprecated simply because the API could not translate that code into STM environment
Note that all I2C code in arduino IDE must have function onRequest in order for high level controller/master to be able to detect the device.
