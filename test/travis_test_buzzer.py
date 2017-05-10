#!/usr/bin/env python
#encoding: utf8

import rospy, unittest, rostest, rosnode, time, actionlib
from std_msgs.msg import UInt16
from pimouse_ros.msg import MusicAction, MusicResult, MusicFeedback, MusicGoal

class BuzzerTest(unittest.TestCase):
	def setUp(self):
		# make action client
		self.client = actionlib.SimpleActionClient("music", MusicAction)
		# make record buffer for writing buzzer device
		self.device_values = []

	def test_node_exist(self):
		nodes = rosnode.get_node_names()
		self.assertIn('/buzzer', nodes, "node does not exist")

	def test_put_value(self):
		pub = rospy.Publisher('/buzzer', UInt16)
		for i in range(10):
			pub.publish(1234)
			time.sleep(0.1)
		with open("/dev/rtbuzzer0","r") as f:
			data = f.readline()
			self.assertEqual(data, "1234\n", "value does not written to rtbuzzer0")

	def test_music(self):
		goal = MusicGoal()
		goal.freqs = [100, 200, 300, 0]
		goal.durations = [2, 2, 2, 2]
		
		# normal test
		self.device_values = []
		self.client.wait_for_server() # wait server wake up
		self.client.send_goal(goal, feedback_cb=self.feedback_cb)
		self.client.wait_for_result() # wait server execute result

		self.assertTrue(self.client.get_result(), "invalid result")
		# self.assertEqual(goal.freqs, self.device_values, "invalid feedback:" + ",".join([str(e) for e in self.device_values])) 
		
		#preempt test
		self.device_values = []
		# self.client.wait_for_server() # dont need to wait server wake up
		self.client.send_goal(goal, feedback_cb=self.feedback_cb)
		self.client.wait_for_result(rospy.Duration.from_sec(0.5)) # wait server execute result

		self.assertFalse(self.client.get_result(), "preemption is requested but result is not correct")
		self.assertFalse(goal.freqs == self.device_values, "preemption is requested but request freq execute all") 
		
	def feedback_cb(self, feedback):pass
		# with open("/dev/rtbuzzer0", 'r') as f:
			# data = f.readline()
			# self.device_values.append(int(data.rstrip()))

if __name__ == '__main__':
	time.sleep(3)
	rospy.init_node('travis_test_buzzer')
	rostest.rosrun('pimouse_ros', 'travis_test_buzzer', BuzzerTest)
