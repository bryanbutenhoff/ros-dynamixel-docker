#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt64

class DataRunner:
  count = 500
  count_rate = 10

  def get_data(self):
    self.count += self.count_rate
    if self.count >= 900 or self.count <= 100:
      self.count_rate = -self.count_rate
    return self.count

def controller():
  pub = rospy.Publisher('servo_command', UInt64, queue_size=10)
  rospy.init_node('controller', anonymous=True)
  rate = rospy.Rate(10) # 1hz
  dataRunner = DataRunner()
  while not rospy.is_shutdown():
    data = dataRunner.get_data()
    rospy.loginfo("Sending: %d" % data)
    pub.publish(data)
    rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
