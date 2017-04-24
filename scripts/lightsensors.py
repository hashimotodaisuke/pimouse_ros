#!/usr/bin/env python
#encoding: utf8
import sys, rospy
from pimouse_ros.msg import LightSensorValues

def get_freq():
	f = rospy.get_param('lightsensors_freq',10)
	try:
		if f <= 0.0:
			raise Exception()
	except:
		rospy.logerr("value error: lightsensors_freq")
		sys.exit(1)

	return f

if __name__ == '__main__':
	devfile = '/dev/rtlightsensor0'
	rospy.init_node('lightsensors')
	# queue size = 1 because it don't need buffer old data for subscriber
	pub = rospy.Publisher('lightsensors', LightSensorValues, queue_size=1)
	
	freq = get_freq()
	# make 10 Hz rate object 
	# while roop is execute 10 time/ 1 second (consider execute time of this process
	# rate = rospy.Rate(10)
	rate = rospy.Rate(freq)
	while not rospy.is_shutdown():
		try:
			with open(devfile, 'r') as f:
				# read and separate data, then save data[3]
				data = f.readline().split()
				# change charactor to integer
				data = [ int(e) for e in data]
				d = LightSensorValues()
				d.right_forward = data[0]
				d.right_side = data[1]
				d.left_side = data[2]
				d.left_forward = data[3]
				d.sum_all = sum(data)
				d.sum_forward = d.right_forward + d.left_forward
				pub.publish(d)
		except IOError:
			rospy.logerr("cannot open " + devfile)

		f = get_freq()
		if f != freq:
			freq = f
			rate = rospy.Rate(freq)

		rate.sleep()
	
