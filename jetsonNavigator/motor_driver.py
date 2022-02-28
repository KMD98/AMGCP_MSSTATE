#!/usr/bin/env python3
from smbus2 import SMBus
import rospy
import numpy as np
from ros_essentials_cpp.msg import motor_odometry

class MotorDriver:
	def __init__(self):
		self.addr = 0x6 # bus address
		self.bus = SMBus(1) # indicates /dev/ic2-1
		self.speed_data = np.array([0,0,0,0])
		rospy.init_node('motor_driver',anonymous = True)
		self.node_name = rospy.get_name()
		rospy.loginfo("Started node %s" % self.node_name)
		rospy.Subscriber("motor_speeds",motor_odometry,self.setspeed_callback)
	
	def setspeed_callback(self,motor_data):
		self.speed_data[0] = motor_data.driver_side
		self.speed_data[1] = motor_data.driver_dir
		self.speed_data[2] = motor_data.passenger_side
		self.speed_data[3] = motor_data.passenger_dir
		self.bus.write_i2c_block_data(self.addr,1,self.speed_data)

	def spin(self):
		rospy.spin() #program is only being called to write the message on i2c. It does not do anything else and is stuck in this loop



if __name__ == '__main__':
	try:
		driver = MotorDriver()
		driver.spin()
		rospy.login("terminated")
	except rospy.ROSInterruptException:
		pass
