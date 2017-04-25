#!/usr/bin/env python
#encding: utf8
import unittest, rostest
import rosnode, rospy
import time
from pimouse_ros.msg import LightSensorValues

class LightsensorTest(unittest.TestCase):  
	#inheritant unittest.TestCase for using function assertXxx
	def setUp(self):
		self.count = 0
		rospy.Subscriber('/lightsensors', LightSensorValues, self.callback)
		self.values = LightSensorValues()

	def callback(self, data):
		self.count +=1
		self.values = data

	def check_values(self,lf,ls,rs,rf):
		vs = self.values
		self.assertEqual(vs.left_forward,  lf, "diffrent value: left_forward")
		self.assertEqual(vs.left_side,     ls, "diffrent value: left_side")
		self.assertEqual(vs.right_side,    rs, "diffrent value: right_side")
		self.assertEqual(vs.right_forward, rf, "diffrent value: right_forward")
		self.assertEqual(vs.sum_all,       lf+ls+rs+rf, "diffrent value: sum_all")
		self.assertEqual(vs.sum_forward,   lf+rf, "diffrent value: sum_forward")
	
	def test_node_exist(self):
		nodes = rosnode.get_node_names()
		self.assertIn('/lightsensors', nodes, "lightsensors node does not exist")

	def test_get_value(self):
		rospy.get_param('lightsensors_freq' ,10)
		time.sleep(2)
		devfile = "/dev/rtlightsensor0"
		with open (devfile, "w") as f:
			try:
				f.write("-1 0 123 4321\n")
			except IOError:
				rospy.logerr("can not write to" + devfile)
		time.sleep(3)
		#confirm callback function called one time at least and get value
		self.assertFalse(self.count==0, "cannot subscribe the topic")
		self.check_values(4321, 123, 0, -1)

	def test_change_parameter(self):
		rospy.set_param('lightsensors_freq', 1)
		time.sleep(2)
		c_prev = self.count
		time.sleep(3)
		self.assertTrue(self.count < c_prev + 4, "freq does not change")
		self.assertFalse(self.count == c_prev, "subscriber is sttopped")

if __name__ == '__main__':
	time.sleep(3)
	rospy.init_node('travis_test_lightsensors')
	rostest.rosrun('pimouse_ros', 'travis_test_lightsensors', LightsensorTest)
			
		
		
