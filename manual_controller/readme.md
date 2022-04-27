# Purpose
- This folder contain configuration files and the code for the manual controller in the ROS Network
- The code in this folder will be placed inside ROS_jetson folder and ROS_network folder
- Deploy the manual controller on an stm32 black pill. 
- The manual receiver is a rosnode that can only be run on a jetson nano or an embedded computer running Ubuntu. To use the receiver, the rosnode must be placed inside the catkin package with the rest of the navigation stack.

# Malduino 'can't open device "/dev/ttyACM0": Device or resource busy'  or 'dev/ttyl/USB0' on Ubuntu
- common problem when deploying the manual receiver or any code that runs i2c or read from serial, the exception will be thrown and require sudo permission.
- The problem can be resolve by adding dialout and i2c to group. Follow the instructions below to add.
## Set Permissions

Add your user to the 'dialout' group

```bash
sudo usermod -aG dialout yourusername
```

(optional) Verify:

```bash
groups
```

'dialout' should be in the included in the list of groups.

## Create udev Rule

```bash
sudo vi /etc/udev/rules.d/77-arduino.rules
```

File content:

```text
ATTRS{idVendor}=="1b4f", ENV{ID_MM_DEVICE_IGNORE}="1"
```
