#!/usr/bin/env python

# Author Bryan Butenhoff

# This has been created by modifying the following example from ROBOTIS 
# *********     Read and Write Example      *********
# *********      Protocol Version 1.0       *********

# ROS Specific
import rospy
from std_msgs.msg import UInt64

# Script Specific
from dynamixel_sdk import *

class DynamixelServo:

  ADDR_MX_TORQUE_ENABLE      = 18
  ADDR_MX_GOAL_POSITION      = 30
  ADDR_MX_PRESENT_POSITION   = 36

  PROTOCOL_VERSION            = 1.0

  DXL_ID                      = 6
  BAUDRATE                    = 1000000
  DEVICENAME                  = '/dev/ttyUSB0'

  TORQUE_ENABLE               = 1
  TORQUE_DISABLE              = 0
  DXL_MINIMUM_POSITION_VALUE  = 10
  DXL_MAXIMUM_POSITION_VALUE  = 1000
  DXL_MOVING_STATUS_THRESHOLD = 10

  def __init__(self):
    self.portHandler = PortHandler(self.DEVICENAME)
    self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

    if not self.portHandler.openPort():
      print("Failed to open the port")
      quit()
    if not self.portHandler.setBaudRate(self.BAUDRATE):
      print("Failed to change the baudrate")
      quit()

  # Close port on destruction
  def __del__(self):
    self.portHandler.closePort()

  def check_comm_result(self, dxl_comm_result, dxl_error):
    if dxl_comm_result != COMM_SUCCESS:
      print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
      print("%s" % self.packetHandler.getRxPacketError(dxl_error))

  def write_one_byte(self, address, value):
    dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, address, value)
    self.check_comm_result(dxl_comm_result, dxl_error)

  def write_two_bytes(self, address, value):
    dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, address, value)
    self.check_comm_result(dxl_comm_result, dxl_error)

  def read_one_bytes(self, address):
    dxl_response, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, self.DXL_ID, address)
    self.check_comm_result(dxl_comm_result, dxl_error)
    return dxl_response

  def read_two_bytes(self, address):
    dxl_response, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, address)
    self.check_comm_result(dxl_comm_result, dxl_error)
    return dxl_response

  def enable_torque(self):
    self.write_one_byte(self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE)

  def disable_torque(self):
    self.write_one_byte(self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_DISABLE)

# ROS METHODS
  def run(self):
    rospy.init_node('servo', anonymous=True)
    rospy.Subscriber("servo_command", UInt64, self.callback)
    rospy.spin()

  def callback(self, data):
    rospy.loginfo(rospy.get_caller_id() + "GoalPos:%03d", data.data)
    self.write_two_bytes(self.ADDR_MX_GOAL_POSITION, data.data)

## STANDALONE
#  def run(self, goal_position):
#    self.write_two_bytes(self.ADDR_MX_GOAL_POSITION, goal_position)
#    
#    while 1:
#      dxl_present_position = self.read_two_bytes(self.ADDR_MX_PRESENT_POSITION)
#    
#      print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (self.DXL_ID, goal_position, dxl_present_position))
#
#      if not abs(goal_position - dxl_present_position) > self.DXL_MOVING_STATUS_THRESHOLD:
#        break

if __name__ == "__main__":
  my_servo = DynamixelServo()
  try:
    my_servo.enable_torque()

# ROS METHODS
    my_servo.run()
    my_servo.disable_torque()
  except rospy.ROSInterruptException:
    my_servo.disable_torque()

## STANDALONE
#    my_servo.run(1000)
#    my_servo.run(10)
#    my_servo.disable_torque()
#  except:
#    my_servo.disable_torque()
