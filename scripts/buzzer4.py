#!/usr/bin/env python
#encoding: utf8

import rospy, actionlib
from std_msgs.msg import UInt16
from pimouse_ros.msg import MusicAction, MusicResult, MusicFeedback
# refer action/Music.action file. 
# MusicAction   uint16[], float32[]  this is goal
# ---
# MusicRessult  bool                 this is result
# ---
# MusicFeedback uint32               this is feedback

def write_freq(hz=0):
	bfile = "/dev/rtbuzzer0"
	try:
		# call close() when it exists with block. so, dont need close()
		with open(bfile, "w") as f:
			f.write(str(hz)+"\n")
	except IOError:
		rospy.logger("cant write to " + bfile)

def recv_buzzer(data):
	write_freq(data.data)

def exec_music(goal):pass

if __name__ == '__main__':
	# register client node "buzzer" with the master node
	rospy.init_node('buzzer')

	# register as a sbuscriber to a special topic
	rospy.Subscriber("buzzer", UInt16, recv_buzzer)

	# make actionlib instance and call start method to start action server
	# exec_music is exectute callback function
	music = actionlib.SimpleActionServer('music', MusicAction, exec_music, False)
	music.start()

	rospy.on_shutdown(write_freq)
	# keep this node until ROS node is shutdown
	# because this node's subscriber can work at all time
	rospy.spin()

