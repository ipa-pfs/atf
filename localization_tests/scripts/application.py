#!/usr/bin/python
import unittest
import rospy
import rostest
import tf
import math
import sys
import rosbag

from simple_script_server import *

from atf_core import ATF

class Application:
    def __init__(self):
        # ATF code
        self.atf = ATF()

        # native app code
        self.pub_freq = 20.0 # Hz
        self.bag = rospy.get_param('bag')
    def state_transition_cb(self, msg):
        # TODO sm feedback on state change [eg: ERROR, READY, BUSY ...]
        rospy.loginfo("callback")
        if 'Done.' in msg.msg:
            self.ready_for_scenario = True
    def execute(self):

        # small testblock (circle r=0.5, time=3)
        rospy.sleep(1)
        self.atf.start("testblock_small")
        bag = rosbag.Bag(self.bag)
        rospy.sleep(80)
        self.atf.stop("testblock_small")
        self.atf.shutdown()

    def wait_for_robot_ready(self):
        rospy.loginfo("wait for robot to be ready")
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.ready_for_scenario:
                break
        r.sleep()

class Test(unittest.TestCase):
    def setUp(self):
        self.app = Application()

    def tearDown(self):
        pass

    def test_Recording(self):
        self.app.execute()

if __name__ == '__main__':
    rospy.init_node('test_name')
    if "standalone" in sys.argv:
        app = Application()
        app.execute()
    else:
        rostest.rosrun('application', 'recording', Test, sysargs=None)
