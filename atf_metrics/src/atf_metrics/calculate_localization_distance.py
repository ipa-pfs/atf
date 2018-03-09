#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf
import math

class CalculateLocalizationDistanceParamHandler:
    def __init__(self):
        """
        Class for returning the corresponding metric class with the given parameter.
        """
        pass

    def parse_parameter(selfself, testblock_name, params):
        """
        Method that returns the metric method with the given parameter.
        :param params: Parameter
        """
        metrics = []
        if type(params) is not list:
            rospy.logerr("metric config not a list")
            return False

        for metric in params:
            try:
                groundtruth = metric["groundtruth"]
                groundtruth_epsilon = metric["groundtruth_epsilon"]
            except (TypeError, KeyError):
                rospy.logwarn("No groundtruth parameters given, skipping groundtruth evaluation for metric 'localization_distance' in testblock '%s'", testblock_name)
                groundtruth = None
                groundtruth_epsilon = None
            try:
                groundtruth_topic = metric["groundtruth_topic"]
                metrics.append(
                    CalculateLocalizationDistance(None, None, None,
                                                  groundtruth_topic))
            except (TypeError, KeyError):
                rospy.logwarn("No groundtruth topic given in testblock '%s' try to use tf evaluation", testblock_name)
                groundtruth_topic = None
            try:
                groundtruth_frame = metric["ground_truth_frame"]
                root_frame = metric["root_frame"]
                measured_frame = metric["measured_frame"]

                metrics.append(
                    CalculateLocalizationDistance(root_frame, measured_frame, groundtruth_frame,
                                                  None))
            except (TypeError, KeyError):
                rospy.logwarn("No groundtruth frame given in testblock '%s'", testblock_name)
                groundtruth_frame = None
        return metrics



class CalculateLocalizationDistance:
    def __init__(self, root_frame, measured_frame, groundtruth_frame, groundtruth_topic):
        """
        Class for calculating the distance between the root_frame-measured_frame  pose and if available the
        groundtruth_topic pose or the root_frame-groundtruth_frame pose
        """
        self.active = False
        self.root_frame = root_frame
        self.measured_frame = measured_frame
        self.groundtruth_topic = groundtruth_topic
        self.groundtruth_frame = groundtruth_frame
        self.tf_sampling_freq = 20.0
        self.finished = False
        self.delta_trans = {}
        self.count = 0
        self.listener = tf.TransformListener()

        if groundtruth_topic is not None:
            rospy.Subscriber(groundtruth_topic, Odometry, self.groundtruth_callback)
        elif groundtruth_frame is not None:
            rospy.Timer(rospy.Duration.from_sec(1 / self.tf_sampling_freq), self.record_tf)
    def start(self, timestamp):
        self.active = True

    def stop(self, timestamp):
        self.active = False
        self.finished = True

    def pause(self, timestamp):
        self.active = False
        self.first_value = True

    def purge(self, timestamp):
        pass

    def groundtruth_callback(self, msg):
        if (self.active):
            try:
                self.listener.waitForTransform(msg.header.frame_id,
                                               msg.child_frame_id,
                                               rospy.Time(0),
                                               rospy.Duration.from_sec(1 /(2* self.tf_sampling_freq)))
                (trans, rot) = self.listener.lookupTransform(msg.header.frame_id, msg.child_frame_id, rospy.Time(0))
                delta_trans = math.sqrt((msg.pose.pose.position.x - trans[0])**2 + (msg.pose.pose.position.y - trans[1])**2)
                rospy.loginfo(delta_trans)
                self.delta_trans[self.count] = delta_trans
                self.count = self.count + 1
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                rospy.logwarn(e)


    def record_tf(self):
        pass

    def get_result(self):
        groundtruth_result = None
        details = {"root_frame": self.root_frame, "measured_frame": self.measured_frame, "topic": self.groundtruth_topic }
        if self.finished:
            #print "Distances: " + str(self.delta_trans) + " Dist Count: " + str(self.count)
            data = self.delta_trans
            print data
            return "localization_distance", data, None, None, None, details
        else:
            return False

