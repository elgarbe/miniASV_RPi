#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from math import cos, sin
import numpy as np
from geometry_msgs.msg import TwistWithCovarianceStamped 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Imu

# Constants that may change in multiple places
MSG_QUEUE_MAXLEN = 50

class GPS_vel_translate:

    def __init__(self):

        self.gps_vel_body_pub = rospy.Publisher("/gps_vel_body", TwistWithCovarianceStamped, queue_size=1)

        # Me suscribo al tópico donde se envia la velocidad 
        self.fix_vel_sub = rospy.Subscriber("/f9p/ublox/fix_velocity", TwistWithCovarianceStamped , self.gps_vel_cb)
        self.imu_data_sub = rospy.Subscriber("/chori/imu/data", Imu , self.imu_cb)

        rospy.loginfo("[GPS vel] Translating GPS velocity fix from ENU to body")
        self.heading = 0.0

    def gps_vel_cb(self, msg):
        enu_vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y]

        Rz_matrix = np.array([
                [cos(self.heading), -sin(self.heading)],
                [sin(self.heading), cos(self.heading)]], 
                dtype='float')

        body_vel = np.dot(Rz_matrix.transpose(), enu_vel)

        new_msg = TwistWithCovarianceStamped()
        new_msg.header = msg.header
        # Roto la velocidad de ENU al Body usando el Heading
        new_msg.twist.twist.linear.x = body_vel[0]
        new_msg.twist.twist.linear.y = body_vel[1]
        new_msg.twist.twist.linear.z = msg.twist.twist.linear.z
        # Publico la velocidad en ternas de Body
        self.gps_vel_body_pub.publish(new_msg)        
        # Vx = msg.twist.twist.linear.x
        # Vy = msg.twist.twist.linear.y
        # ch = math.cos(self.heading)
        # sh = math.sin(self.heading)
        # new_msg = TwistWithCovarianceStamped()

        # new_msg.header = msg.header
        # # Roto la velocidad de ENU al Body usando el Heading  
        # new_msg.twist.twist.linear.x = Vx * ch - Vy * sh
        # new_msg.twist.twist.linear.y = Vx * sh + Vy * ch
        # new_msg.twist.twist.linear.z = msg.twist.twist.linear.z
        # new_msg.twist.twist.angular = msg.twist.twist.angular
        # new_msg.twist.covariance = msg.twist.covariance
        # # Publico la velocidad en ternas de Body
        # self.gps_vel_body_pub.publish(new_msg)

    def imu_cb(self, dato):
        orientation_q = dato.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.heading =  yaw #* 180/3.141592

    def shutdown(self):
        """Unregisters publishers and subscribers and shutdowns timers"""
        self.fix_vel_sub.unregister()
        self.imu_data_sub.unregister()
        self.gps_vel_body_pub.unregister()
        rospy.loginfo("[GPS vel] Sayonara GPS vel. Nos vemo' en Disney.")

def main(args):
    rospy.init_node('GPS_vel_translate', anonymous=True)
    node = GPS_vel_translate()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    node.shutdown()

if __name__ == '__main__':
    main(sys.argv)
