#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt64

def controller():
    count_rate = 10
    count = 500
    pub = rospy.Publisher('servo_command', UInt64, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(10) # 1hz
    while not rospy.is_shutdown():
        if count >= 900 or count <= 100:
          count_rate = -count_rate
        data = count
        rospy.loginfo("Sending: %d" % data)
        pub.publish(data)
        rate.sleep()
        count += count_rate

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
