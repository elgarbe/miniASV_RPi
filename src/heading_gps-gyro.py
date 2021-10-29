#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Imu
from ublox_msgs.msg import NavRELPOSNED9

# Constants that may change in multiple places
MSG_QUEUE_MAXLEN = 50
DIST_GPS_ANNTENA = 70       # Distancia real entre las antenas

class HeadingNode(object):

    def __init__(self):
        
        imu_topic = rospy.get_param('~imu_topic', "/chori/imu/data")
        imu_filt_topic = rospy.get_param('~imu_filt_topic', "/imu/data")
        gyro_topic = rospy.get_param('~gyro_topic', "/chori/imu/data_raw")
        gps_topic = rospy.get_param('~gps_topic', "/f9p/ublox/navrelposned")

        gyro_heading_topic = rospy.get_param('~gyro_heading_topic', "/chori/gyro_heading")
        gps_heading_topic = rospy.get_param('~gps_heading_topic', "/chori/gps_heading")

        rospy.loginfo("[Heading] Waiting for imu message...")
        imu_msg = rospy.wait_for_message(gyro_topic, Imu)
        self.prev_time = imu_msg.header.stamp.to_sec()

        # Ganancia del filtro complementario
        self.filt_alfa = 0.97
        # Headind del filtro complementario
        self.heading = 0.0
        self.roll = 0.0
        self.pitch = 0.0

        # Heading del GPS, RELPOSNED
        self.gps_heading = 0.0
        # Heading integrando el gyróscopo
        self.gyro_heading = 0.0
        # Contador de mediciones correctas del GPS
        self.cnt_meas = 0
        # Bandera para indicar que la medición del GPS es válida
        self.valid_gps_heading = False

        # Me suscribo al tópico donde se envia la velocidad 
        self.gps_sub = rospy.Subscriber(gps_topic, NavRELPOSNED9 , self.gps_cb)
        self.gyro_sub = rospy.Subscriber(gyro_topic, Imu , self.imu_cb)
        self.imu_filt_sub = rospy.Subscriber(imu_filt_topic, Imu , self.imu_filt_cb)

        self.gyro_heading_pub = rospy.Publisher(gyro_heading_topic, Vector3, queue_size=MSG_QUEUE_MAXLEN)
        rospy.loginfo("[Heading] Will publish gyro heading to topic %s", gyro_heading_topic)

        self.gps_heading_pub = rospy.Publisher(gps_heading_topic, Vector3, queue_size=MSG_QUEUE_MAXLEN)
        rospy.loginfo("[Heading] Will publish GPS heading to topic %s", gps_heading_topic)

        self.imu_pub = rospy.Publisher(imu_topic, Imu, queue_size=MSG_QUEUE_MAXLEN)
        rospy.loginfo("[Heading] Will publish IMU with orientation to topic %s", imu_topic)

    def gps_cb(self, msg):
        flags = msg.flags
        heading = msg.relPosHeading 
        length = msg.relPosLength + (msg.relPosHPLength * 1e-2)
        fix_type = ''
        # if (flags & 1):
        #     print('FIX OK')
        # if (flags & 4):
        #     print('Valid measurement')
        if (flags & 8):
            fix_type = 'Float'
        if (flags & 16):
            fix_type = 'Fixed'
        # if (flags & 32):
        #     print('Moving Base config')
        # if (flags & 256):
        #     print('Valid heading')
        
        # Necesito al menos 10 mediciones correctas seguidas 
        # de distancia entre antenas para considerar que esta midiendo bien
        # Tambien pido que el currierSolution sea Fixed
        lenght_error = abs(length - DIST_GPS_ANNTENA)
        if (lenght_error < 3) & ((flags & 16) == 16):
            self.cnt_meas = self.cnt_meas + 1
            if self.cnt_meas == 10:
                rospy.loginfo("[Heading] GPS is reporting heading...")
                self.valid_gps_heading = True
        else:
            self.cnt_meas = 0
            self.valid_gps_heading = False
            rospy.loginfo("[Heading] GPS lost heading. Length error: %s [cm], Fix type: %s", lenght_error, fix_type)
        if self.valid_gps_heading:
            # Obtengo el heading del GPS y lo paso a Grados
            self.gps_heading = 270 - heading * 1e-5 
            # Publico el dato para debug
            dato = Vector3()
            dato.x = self.gps_heading
            self.gps_heading_pub.publish(dato)

    def imu_cb(self, imu_msg):
        dt = imu_msg.header.stamp.to_sec() - self.prev_time
        self.prev_time = imu_msg.header.stamp.to_sec()
        delta_ang = imu_msg.angular_velocity.z * dt
        self.gyro_heading = self.gyro_heading + delta_ang

        dato = Vector3()
        dato.x = self.gyro_heading * 180/3.141592
        self.gyro_heading_pub.publish(dato)

        self.heading = self.filt_alfa * (self.heading + delta_ang) + self.gps_heading*3.141592/180 * (1-self.filt_alfa)
        # print(self.heading * 180/3.141592)

        quat = quaternion_from_euler(self.roll, self.pitch, self.heading)
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = imu_msg.header.frame_id
        msg.orientation.x = quat[0] 
        msg.orientation.y = quat[1] 
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        # Meto la covarianza de orientación
        orient_cov = np.reshape(np.array(0.001 * np.identity(3, dtype=float)),(9)).tolist()
        acc_gyr_cov = np.reshape(np.array(0.1 * np.identity(3, dtype=float)),(9)).tolist()
        msg.orientation_covariance = orient_cov#imu_msg.orientation_covariance
        msg.linear_acceleration = imu_msg.linear_acceleration
        msg.linear_acceleration_covariance = acc_gyr_cov #imu_msg.linear_acceleration_covariance
        msg.angular_velocity = imu_msg.angular_velocity
        msg.angular_velocity_covariance = orient_cov #imu_msg.angular_velocity_covariance

        self.imu_pub.publish(msg)        
        # orient_cov = np.reshape(np.array(0.1 * np.identity(3, dtype=float)),(9)).tolist()
        # msg.orientation_covariance = orient_cov#imu_msg.orientation_covariance
        # msg.linear_acceleration = imu_msg.linear_acceleration
        # msg.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
        # msg.angular_velocity = imu_msg.angular_velocity
        # msg.angular_velocity_covariance = imu_msg.angular_velocity_covariance

        # self.imu_pub.publish(msg)

    def imu_filt_cb(self, imu_msg):
        quat = (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
        self.roll, self.pitch, yaw = euler_from_quaternion(quat)


    def shutdown(self):
        self.gps_sub.unregister()
        self.gyro_sub.unregister()
        self.imu_filt_sub.unregister()
        self.gyro_heading_pub.unregister()
        self.imu_pub.unregister()
        self.gps_heading_pub.unregister()

def main(args):
    rospy.init_node('Heading_gps_gyro', anonymous=True, log_level=rospy.INFO)
    node = HeadingNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    node.shutdown

if __name__ == '__main__':
    main(sys.argv)
