#!/usr/bin/env python

# Author Bryan Butenhoff

# This has been created by modifying the following example from ROBOTIS 
# *********     Read and Write Example      *********
# *********      Protocol Version 1.0       *********

# ROS Specific
import rospy
from std_msgs.msg import UInt64

import os
from servowrapper import Usb2Dynamixel
from servowrapper import AX12Servo
from functools import partial

def callback(servo, usb_dynamixel, data):
  servo.set_goal_position(usb_dynamixel, data.data)

def run(servo, usb_dynamixel):
  rospy.init_node('servo', anonymous=True)
  rospy.Subscriber("servo_command", UInt64, partial(callback, servo, usb_dynamixel))
  rospy.spin()

if __name__ == "__main__":

  # Create a Usb2Dynamixel for communication
  device_name = os.environ['TTL_DEVICE']
  baud_rate = 1000000
  protocol_version = 1.0
  usb_dynamixel = Usb2Dynamixel(device_name, baud_rate, protocol_version)

  # Create a servo instance for defined ID
  id = int(os.environ['DXL_IDS'])
  servo = AX12Servo(id)

  try:
    servo.enable_torque(usb_dynamixel)
    run(servo, usb_dynamixel)
    servo.disable_torque(usb_dynamixel)
  except rospy.ROSInterruptException:
    servo.disable_torque(usb_dynamixel)
