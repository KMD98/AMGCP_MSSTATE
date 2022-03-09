#!/usr/bin/env python
#from smbus import SMBus
import rospy
import numpy as np
from ros_essentials_cpp.msg import motor_odometry,encoder_odometry

class MotorDriver:
	def __init__(self):
		self.addr = 0x6 # bus address
		self.bus = SMBus(1) # indicates /dev/ic2-1
		self.speed_data = np.array([0,0,0,0]) #array to send to i2c
		self.previous_speeds = np.zeros(4) #array to compare previous speed to current
		rospy.init_node('motor_driver',anonymous = True)
		self.node_name = rospy.get_name()
		rospy.loginfo("Started node %s" % self.node_name)
		#Declare the topic that it is subscribed to, in this case the desired motor speeds
		rospy.Subscriber("motor_speeds",motor_odometry,self.setspeed_callback)
		rospy.Subscriber("manual_speeds",motor_odometry,self.manual_callback)
		#Declare the topic that it is going to publish to, in this case the topic have real encoder readings
		self.pub = rospy.Publisher('feedback_rpm',encoder_odometry,queue_size=10)
		self.rate = rospy.Rate(10) #set to 10Hz

	def setspeed_callback(self,motor_data):
		self.speed_data[0] = motor_data.driver_side
		self.speed_data[1] = motor_data.driver_dir
		self.speed_data[2] = motor_data.passenger_side
		self.speed_data[3] = motor_data.passenger_dir
		if np.sum(self.speed_data) != np.sum(self.previous_speeds): #if not the same then send new desired speed commands
			self.bus.write_i2c_block_data(self.addr,1,self.speed_data)
			for i in range(0,4):
				self.previous_speeds[i] = self.speed_data[i] #initialize "previous array" with current values for next comparison

	def manual_callback(self,manual_data):
		self.speed_data[0] = manual_data.driver_side
		self.speed_data[1] = manual_data.driver_dir
		self.speed_data[2] = manual_data.passenger_side
		self.speed_data[3] = manual_data.passenger_dir
		if np.sum(self.speed_data) != np.sum(self.previous_speeds): #if not the same then send new desired speed commands
			self.bus.write_i2c_block_data(self.addr,1,self.speed_data)
			for i in range(0,4):
				self.previous_speeds[i] = self.speed_data[i] #initialize "previous array" with current values for next comparison
				
	def get_rpm(self):
		return self.bus.read_i2c_block_data(self.addr,0,8)

	def spin(self):
		while not rospy.is_shutdown():
			actual_rpm = encoder_odometry() #declare the message object
			temp = get_rpm() #read actual encoder data
			actual_rpm.target_driver = temp[0]
			actual_rpm.dir_driver= temp[1]
			actual_rpm.target_pass = temp[2]
			actual_rpm.dir_pass= temp[3]
			actual_rpm.enc1 = temp[4]
			actual_rpm.enc2= temp[5]
			actual_rpm.enc3 = temp[6]
			actual_rpm.enc4= temp[7]
			self.pub(actual_rpm)
			self.rate.sleep()

if __name__ == '__main__':
	try:
		driver = MotorDriver()
		driver.spin()
		rospy.login("terminated")
	except rospy.ROSInterruptException:
		pass
