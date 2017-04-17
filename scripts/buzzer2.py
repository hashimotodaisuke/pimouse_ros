#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16

def recv_buzzer(data):
	# logging obhect type(UInt16) of data
	rospy.loginfo(type(data))
	# logging data(value) of data
	rospy.loginfo(data.data)

if __name__ == '__main__':
	# register client node "buzzer" with the master node
	rospy.init_node('buzzer')

	# register as a sbuscriber to a special topic
	rospy.Subscriber("buzzer", UInt16, recv_buzzer)

	# keep this node until ROS node is shutdown
	# because this node's subscriber can work at all time
	rospy.spin()

