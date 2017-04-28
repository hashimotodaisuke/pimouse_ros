#!/usr/bin/env python
#encoding utf8
import sys, rospy, math
from pimouse_ros.msg import MotorFreqs
from geometry_msgs.msg import Twist

class Motor():
	def __init__(self):
		# call set_power function. if power on is fail, system exit
		if not self.set_power(True): sys.exit(1)
		# if self.set_power(True)==False: sys.exit(1)

		#set node end function. if this node is end, system calls this function
		rospy.on_shutdown(self.set_power)
		self.sub_raw =     rospy.Subscriber('motor_raw', MotorFreqs, self.callback_raw_freq)
		self.sub_cmd_vel = rospy.Subscriber('cmd_vel',   Twist,      self.callback_cmd_vel)
		self.last_time = rospy.Time.now()
		self.using_cmd_vel = False

	# device access method
	def set_power(self, onoff=False):
		en = "/dev/rtmotoren0"
		try:
			with open(en, 'w') as f:
				# 3 Kouenzan(Ternary operation) of python. if onoff==True then 1\n else 0\n
				f.write("1\n" if onoff else "0\n") 
			self.is_on = onoff
			return True
		except:
			rospy.logerr("cannot write to " + en)
		return False
	
	# device access method
	def set_raw_freq(self, left_hz, right_hz):
		if not self.is_on:
			rospy.logerr("not enpowered")
			return
		try:
			with open("/dev/rtmotor_raw_l0", 'w') as lf, \
			     open("/dev/rtmotor_raw_r0", 'w') as rf:
				lf.write(str(int(round(left_hz))) + "\n")
				rf.write(str(int(round(right_hz))) + "\n")
		except:
			ros.logerr("canmnot write to rtmotor_raw_*")

	def callback_raw_freq(self, message):
		self.set_raw_freq(message.left_hz, message.right_hz)

	def callback_cmd_vel(self, message):
		forward_hz = 80000.0*message.linear.x/(9*math.pi)
		rot_hz = 400.0*message.angular.z/math.pi
		self.set_raw_freq(forward_hz-rot_hz, forward_hz+rot_hz)
		self.using_cmd_vel = True
		self.last_time = rospy.Time.now()

if __name__ == '__main__':
	rospy.init_node('motors')
	m = Motor()

	rate = rospy.Rate(10) # execute loop 10times/sec
	while not rospy.is_shutdown():
		if m.using_cmd_vel and rospy.Time.now().to_sec() - m.last_time.to_sec() >=1.0:
			m.set_raw_freq(0,0)
			m.using_cmd_vel = False
		rate.sleep()

		
