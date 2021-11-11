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
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from std_msgs.msg import Float32

class ReferenceNode(object):
    """Docstring"""
    def __init__(self):
        self.rate = float(rospy.get_param('~rate', '50'))    # 50 Hz
        reference_topic = rospy.get_param('~twist_ref_topic', None)

        # Referencia inicial
        self.ref = np.array([0.0, 0.0])
        # Suscriptores a los valores de referencia deseados
        self.sub_ref_fvel = rospy.Subscriber('ref_fvel', Float32, self.set_ref_fvel)
        self.sub_ref_yr   = rospy.Subscriber('ref_yr', Float32, self.set_ref_yr)

        # Creo el Publisher para la referencia
        self.reference_publisher = rospy.Publisher(reference_topic, TwistStamped, queue_size=10)
        rospy.loginfo('[REF] Will publish Twist reference to topic: %s', reference_topic)

        # Creo el timer con el que lo voy a invocar
        self.publish_timer = rospy.Timer(rospy.Duration(1/self.rate), self.publish, oneshot=False)
        rospy.loginfo('[REF] Update rate: %f Hz', self.rate)

        rospy.loginfo('[REF] Publishing miniASV reference')


    def set_ref_fvel(self, x):
        self.ref[0] = float(x.data)

    def set_ref_yr(self, y):
        self.ref[1] = float(y.data)

    def publish(self, event):
        # Armo el mensaje a publicar
        message = TwistStamped()
        message.header.stamp = rospy.Time.now()
        message.twist.linear.x = self.ref[0]
        message.twist.angular.z = self.ref[1]
        # Publico el mensaje
        self.reference_publisher.publish(message)

    def shutdown(self):
        """Unregisters publishers and subscribers and shutdowns timers"""
        self.publish_timer.shutdown()
        self.reference_publisher.unregister()
        self.sub_ref_fvel.unregister()
        self.sub_ref_yr.unregister()
        rospy.loginfo("[REF] Sayonara generador de referencia. Nos vemo' en Disney.")

def main():
    """Entrypoint del nodo"""
    rospy.init_node('asv_ref_twist', anonymous=True, log_level=rospy.INFO)
    node = ReferenceNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[REF] Received Keyboard Interrupt (^C). Shutting down.")

    node.shutdown()

if __name__ == '__main__':
    main()