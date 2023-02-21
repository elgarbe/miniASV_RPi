#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on

@author: elgarbe


"""
# NumPy
import numpy as np

# ROS Python API
import rospy
from topic_tools.srv import *

# Import the messages we're interested in sending and receiving, and having and sharing
from std_msgs.msg import UInt8
from asv_chori.msg import CourseStamped
from geometry_msgs.msg import TwistStamped, WrenchStamped, Vector3, Vector3Stamped

class RadioControlNode(object):
    """Docstring"""
    def __init__(self):
        self.rate = float(rospy.get_param('~rate', '50'))    # 50 Hz
        rc_topic = rospy.get_param('~rc_topic', '/chori/RC')
        ref_course_topic = rospy.get_param('~ref_course_topic', '/radio_control/course_ref')
        ref_twist_topic = rospy.get_param('~ref_twist_topic', '/radio_control/twist_ref')
        ms_topic = rospy.get_param('~ms_topic', '/radio_control/ms')

        rospy.wait_for_service('mux_cmdvel/select')
        self.prev_ms = 0
        
        # Suscriptores
        rospy.loginfo('[RC] Suscribing to Radio Control commands topic: %s', rc_topic)
        self.rc_sub = rospy.Subscriber(rc_topic, Vector3, self.rc_cb)

        # Publishers
        self.ref_course_pub = rospy.Publisher(ref_course_topic, CourseStamped, queue_size=10)
        rospy.loginfo('[RC] Will publish Course reference to topic: %s', ref_course_topic)
        self.ref_twist_pub = rospy.Publisher(ref_twist_topic, TwistStamped, queue_size=10)
        rospy.loginfo('[RC] Will publish Twist reference to topic: %s', ref_twist_topic)

        self.ms_pub = rospy.Publisher(ms_topic, UInt8, queue_size=10)
        rospy.loginfo('[RC] Will publish Mode Switch to topic: %s', ms_topic)

    def rc_cb(self, msg):
        ref_fvel = msg.x
        ref_ms = msg.y
        ref_yaw = msg.z

        self.ms_pub.publish(ref_ms)

        if ref_ms == 2 :
            # El mode Switch indica que el radio control está enviando referencias de Course
            message = CourseStamped()
            message.header.stamp = rospy.Time.now()
            message.yaw = ref_yaw
            message.speed = ref_fvel
            self.ref_course_pub.publish(message)
            if self.prev_ms != 2:
                self.prev_ms = 2
                try:
                    add_two_ints = rospy.ServiceProxy('mux_cmdvel/select', MuxSelect)
                    resp1 = add_two_ints('/yaw_pid/twist_ref')
                    # print(resp1)
                    # return resp1.sum
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)
        if ref_ms == 4 :
            # El mode Switch indica que el radio control está enviando referencias de Twist
            message = TwistStamped()
            message.header.stamp = rospy.Time.now()
            message.twist.linear.x = ref_fvel
            message.twist.angular.z = ref_yaw
            self.ref_twist_pub.publish(message)

            if self.prev_ms != 4:
                self.prev_ms = 4
                try:
                    add_two_ints = rospy.ServiceProxy('mux_cmdvel/select', MuxSelect)
                    resp1 = add_two_ints('/radio_control/twist_ref')
                    # print(resp1)
                    # return resp1.sum
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)

    def shutdown(self):
        """Unregisters publishers and subscribers and shutdowns timers"""
        self.rc_sub.unregister()
        self.ref_course_pub.unregister()
        self.ref_twist_pub.unregister()
        self.ms_pub.unregister()
        rospy.loginfo("[RC] Sayonara Radio Control. Nos vemo' en Disney.")

def main():
    """Entrypoint del nodo"""
    rospy.init_node('radio_control', anonymous=True, log_level=rospy.INFO)
    node = RadioControlNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[RC] Received Keyboard Interrupt (^C). Shutting down.")

    node.shutdown()

if __name__ == '__main__':
    main()