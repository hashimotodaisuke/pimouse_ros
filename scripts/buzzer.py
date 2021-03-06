#!/usr/bin/env python
#encoding: utf8

import rospy, actionlib
from std_msgs.msg import UInt16
from pimouse_ros.msg import MusicAction, MusicResult, MusicFeedback
# refer action/Music.action file. 
# MusicGoal     uint16[], float32[]  this is goal. this is not used in this file. the client use it.
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

def exec_music(goal):
	r = MusicResult()
	fb = MusicFeedback()
	#input value of goal.freqs to i and input arg index of (current) goal.freqs to f 
	for i, f in enumerate(goal.freqs):
		fb.remaining_steps = len(goal.freqs) - i
		music.publish_feedback(fb)
		
		if music.is_preempt_requested():
			write_freq(0)
			r.finished = False
			# occur preemption then call set preempted method and stop music
			music.set_preempted(r)
			return
		
		write_freq(f)
		# sleep for duration
		rospy.sleep(1.0 if i >=len(goal.durations) else goal.durations[i])
	
	# execute all freq then call set_succeeded method 
	r.finished = True
	music.set_succeeded(r)

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

