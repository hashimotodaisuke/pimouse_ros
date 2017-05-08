#!/usr/bin/env python
#encoding: utf8
import unittest, rostest
import rosnode, rospy
import time
from pimouse_ros.msg import MotorFreqs #use MotorFreqs structure
from geometry_msgs.msg import Twist    #use Twist structure
from std_srvs.srv import Trigger, TriggerResponse #use Trigger structure

class MotorTest(unittest.TestCase): #inheritant unittest.TestCase to use assertXXX functions
	def setUp(self):
		rospy.wait_for_service('/motor_on')
		rospy.wait_for_service('/motor_off')
		on = rospy.ServiceProxy('/motor_on', Trigger)
		ret = on()

	def file_check(self, dev, value, message):
		with open("/dev/" + dev, 'r') as f:
			line = f.readline()
			# line = message
			self.assertEqual(line, str(value)+"\n", message)
	def test_node_exist(self):
		nodes = rosnode.get_node_names()
		self.assertIn('/motors', nodes, "node does not exist")

	def test_put_freq(self):
		pub = rospy.Publisher('/motor_raw', MotorFreqs)
		m = MotorFreqs()
		m.left_hz = 123
		m.right_hz = 456
		for i in range(10):
			pub.publish(m)
			time.sleep(0.1)
		self.file_check("rtmotor_raw_l0", m.left_hz, "wrong left value from motor_raw")
		self.file_check("rtmotor_raw_r0", m.right_hz, "wrong right value from motor_raw")

	def test_put_cmd_vel(self):
		pub = rospy.Publisher('/cmd_vel', Twist)
		m = Twist()
		m.linear.x = 0.1414
		m.angular.z = 1.57	#then this speed and w-speed make left_hz 200hz and right_hz 600Hz
		for i in range(10):
			pub.publish(m)
			time.sleep(0.1)
		
		self.file_check("rtmotor_raw_l0", 200, "wrong linear x value from cmd_vel")
		self.file_check("rtmotor_raw_r0", 600, "wrong angular.z  value from cmd_vel")
		
		time.sleep(1.1)
		
		self.file_check("rtmotor_raw_l0", 0, "raw_left doesn't stop after 1[s]")
		self.file_check("rtmotor_raw_r0", 0, "raw_right doesn't stop after 1[s]")
	
	def test_on_off(self):
		off = rospy.ServiceProxy('/motor_off', Trigger)
		ret = off()
		self.assertEqual(ret.success, True, "motor off dose not succeeded")
		self.assertEqual(ret.message, "OFF", "motor off wrong message")
		with open("/dev/rtmotoren0", 'r') as f:
			data = f.readline()
			self.assertEqual(data, "0\n", "wrong value in rtmotor0 at motor off")
		
		on = rospy.ServiceProxy('/motor_on', Trigger)
		ret = on()
		self.assertEqual(ret.success, True, "motor on dose not succeeded")
		self.assertEqual(ret.message, "ON", "motor on wrong message")
		with open("/dev/rtmotoren0", 'r') as f:
			data = f.readline()
			self.assertEqual(data, "0\n", "wrong value in rtmotor0 at motor on")

if __name__ == '__main__':
	rospy.init_node('travis_test_motors')
	rostest.rosrun('pimouse_ros', 'travis_test_motors', MotorTest)
