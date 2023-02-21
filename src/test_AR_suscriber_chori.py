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

# Import the messages we're interested in sending and receiving, and having and sharing
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

class testNode(object):
    """Docstring"""
    def __init__(self):
        self.rate = float(rospy.get_param('~rate', '50'))    # 50 Hz

        pos_local_topic = rospy.get_param('~', '/asv/mavros/global_position/local')
        pos_global_topic = rospy.get_param('~', '/asv/mavros/global_position/global')
        pwm_motors_topic = rospy.get_param('~', '/cmd_pwm')

        self.change_pub = 0

        # Suscriptores
        rospy.loginfo('[RC] Suscribing to Radio Control commands topic: %s', pos_local_topic)
        self.pos_local_sub = rospy.Subscriber(pos_local_topic, Odometry, self.pos_loc_cb)
        rospy.loginfo('[RC] Suscribing to Radio Control commands topic: %s', pos_local_topic)
        self.pos_global_sub = rospy.Subscriber(pos_global_topic, NavSatFix, self.pos_glo_cb)

        # Publishers
        self.pwm_pub = rospy.Publisher(pwm_motors_topic, Vector3, queue_size=10)
        rospy.loginfo('[RC] Will publish Course reference to topic: %s', pwm_motors_topic)

    def pos_glo_cb(self, msg):
        pass

    def pos_loc_cb(self, msg):
        message = Vector3()
        # message.header.stamp = rospy.Time.now()
        self.change_pub += 1
        if (self.change_pub > 100):
            self.change_pub = 0

        if (self.change_pub <= 50):
            print("publicando 1500")
            message.x = 1500
            message.y = 1500
        if (self.change_pub > 50):
            print("publicando 1530")
            message.x = 1530
            message.y = 1530

        self.pwm_pub.publish(message)

    def shutdown(self):
        """Unregisters publishers and subscribers and shutdowns timers"""
        self.pos_local_sub.unregister()
        self.pos_global_sub.unregister()
        self.pwm_pub.unregister()
        rospy.loginfo("[RC] Sayonara Radio Control. Nos vemo' en Disney.")

def main():
    """Entrypoint del nodo"""
    rospy.init_node('test_node', anonymous=True, log_level=rospy.INFO)
    node = testNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[RC] Received Keyboard Interrupt (^C). Shutting down.")

    node.shutdown()

if __name__ == '__main__':
    main()