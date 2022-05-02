#!/usr/bin/python3
#Remember to change python3 to python or vice-versa, depending on the computer this ROS network runs on.
#Do not point python2 to python3 because that will create a filing disaster. Simply change the interpreter requirement
from ros_essentials_cpp.msg import motor_rpm,operation_modes
import serial
import rospy

class controller:
  def __init__(self):
    rospy.init_node('manual_receiver',anonymous=True)
    self.node_name = rospy.get_name()
    rospy.loginfo("Started node %s" % self.node_name)
    self.motor_pub = rospy.Publisher('/control/manual_speeds',motor_rpm,queue_size=10)
    self.operation_pub = rospy.Publisher('/control/mode', operation_modes, queue_size = 10)
    #Initialize serial port
    self.ser = serial.Serial('/dev/ttyUSB0', 19200, 8, 'N', 1, timeout=1)
    #Declare subroutine variables
    self.output = ''
    self.data =[]

  def process_data(self):
    temp = ""
    speeds = []
    sw = []
    for i in range(1,11):
      if self.data[i] != b'/':
        temp = temp + self.data[i].decode('ascii')
      elif self.data[i] == b'/':
        speeds.append(int(temp))
        temp = ""
    self.rpm_sender(speeds)
    self.op_sender()
    #Clear all variables
    self.data =[]
    self.output = ""  

  def rpm_sender(self,received_data):
    #publish the speeds
    move_command = motor_rpm()
    move_command.driver_side = received_data[0]
    move_command.driver_dir = received_data[1]
    move_command.passenger_side = received_data[2]
    move_command.passenger_dir = received_data[3]
    #uncommend loginfo for debugging
    #rospy.loginfo(move_command)
    self.motor_pub.publish(move_command)

  def op_sender(self):
    ops = operation_modes()
    ops.RTK_enable = int(self.data[11].decode('ascii'))
    ops.enable = int(self.data[13].decode('ascii'))
    ops.autonomous = int(self.data[15].decode('ascii'))
    #uncomment loginfo for debuggin
    #rospy.loginfo(ops)
    self.operation_pub.publish(ops)

  def spin(self):
    while not rospy.is_shutdown():
      while self.output != b'!': #we use byte conversion here because data received from xbee is in byte format, and python 3 does not automatically converts char because a char is not a byte unlike c++
        self.output = self.ser.read(1)
      while self.output != b'#':
        self.data.append(self.output)
        self.output = self.ser.read(1)
      self.data.append(self.output)
      self.process_data()


if __name__ == '__main__':
  try:
    manual_controller = controller()
    manual_controller.spin()
    rospy.loginfo("terminated")
  except rospy.ROSInterruptException:
    pass