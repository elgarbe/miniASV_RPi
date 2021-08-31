#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import math
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import MagneticField, Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Constants that may change in multiple places
MSG_QUEUE_MAXLEN = 50

class calc_heading:

    def __init__(self):

        # Me suscribo al tópico donde se envian las imágenes
        self.mag_raw_sub = rospy.Subscriber("/chori/imu/mag_raw", Vector3Stamped, self.mag_raw_cb)
        # Me suscribo al tópico donde se envian las imágenes
        self.mag_raw_sub = rospy.Subscriber("chori/imu/rpy/filtered", Vector3Stamped, self.madw_cb)
        # Me suscribo al tópico donde se envian las imágenes
        self.mag_sub = rospy.Subscriber("/chori/imu/mag", MagneticField, self.mag_cb)
        # Me suscribo al tópico donde se envian las imágenes
        self.mag_sub = rospy.Subscriber("/chori/imu/data_raw", Imu, self.imu_raw_cb)
        # Me suscribo al tópico donde se envian las imágenes
        self.mag_sub = rospy.Subscriber("/chori/imu/data", Imu, self.imu_cb)
        # Me suscribo al tópico donde se envian los datos de odometría
        self.heading_pub   = rospy.Publisher("/heading", Float32, queue_size=10)


    def mag_raw_cb(self, dato):
        mag_x = dato.vector.x - 282.7345
        mag_y = dato.vector.y - -403.7714
        mag_z = dato.vector.z - -612.3731
        heading =  (math.atan2(mag_x, mag_y) -0.1524) * 180/3.141592
        rospy.loginfo("Heading from raw mag: %05.3f", heading)

    def madw_cb(self, dato):
        heading =  (dato.vector.z -0.1524) * 180/3.141592
        rospy.loginfo("Heading from madwick : %05.3f", heading)

    def mag_cb(self, dato):
        mag_x = dato.magnetic_field.x
        mag_y = dato.magnetic_field.y
        mag_z = dato.magnetic_field.z
        heading =  (math.atan2(mag_x, mag_y) -0.1524) * 180/3.141592
        rospy.loginfo("Heading from mag: %05.3f", heading)

    def imu_raw_cb(self, dato):
        orientation_q = dato.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        heading =  (yaw -0.1524) * 180/3.141592
        rospy.loginfo("Heading from Imu raw: %05.3f", heading)

    def imu_cb(self, dato):
        orientation_q = dato.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        heading =  (yaw -0.1524) * 180/3.141592
        rospy.loginfo("Heading from Imu madw: %05.3f", heading)


def main(args):
    ic = calc_heading()
    rospy.init_node('Calc_Heading', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
