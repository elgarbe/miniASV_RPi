#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on

@author: elgarbe


"""
# NumPy
import numpy as np
from math import *
# ROS Python API
import rospy

# Import the messages we're interested in sending and receiving, and having and sharing
from asv_chori.msg import CourseStamped
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
# Libraries for data manipulation
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class GuidanceGPSNode(object):
    """Docstring"""
    def __init__(self):
        self.rate = float(rospy.get_param('~rate', '50'))    # 50 Hz
        curr_pos_topic = rospy.get_param('~current_position_topic', 'f9p/ublox/fix')
        dest_pos_topic = rospy.get_param('~destination_position_topic', 'guidance/destination')
        reference_topic = rospy.get_param('~course_ref_topic', 'control/course_ref')
        velocity_topic = rospy.get_param('~velocity_topic', 'odometry/filtered')

        # Suscriptores a los valores de referencia deseados
        self.curr_pos_sub = rospy.Subscriber(curr_pos_topic, NavSatFix, self.curr_pos_cb)
        self.dest_pos_sub = rospy.Subscriber(dest_pos_topic, Pose2D, self.dest_pos_cb)
        # Me suscribo al tópico donde se envian los datos de velocidad
        self.velo_subs = rospy.Subscriber(velocity_topic, Odometry, self.curr_heading_cb)
        rospy.loginfo('[GUID] Subscribing to ASV velocity topic: %s', velocity_topic)

        # Creo el Publisher para la referencia
        self.reference_publisher = rospy.Publisher(reference_topic, CourseStamped, queue_size=10)
        rospy.loginfo('[GUID] Will publish Course reference to topic: %s', reference_topic)

        # Creo el timer con el que lo voy a invocar
        self.publish_timer = rospy.Timer(rospy.Duration(1/self.rate), self.publish, oneshot=False)
        rospy.loginfo('[GUID] Update rate: %f Hz', self.rate)

        rospy.loginfo('[GUID] Publishing miniASV reference')

        self.curr_lat = 0
        self.curr_lon = 0
        self.dest_lat = 0
        self.dest_lon = 0
        self.curr_y = 0

    def curr_pos_cb(self, msg):
        self.curr_lat = msg.latitude
        self.curr_lon = msg.longitude

    def dest_pos_cb(self, msg):
        self.dest_lat = msg.x
        self.dest_lon = msg.y

    def curr_heading_cb(self, msg):
        quat = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.curr_y = euler_from_quaternion(quat)

    def publish(self, event):
        dist = self.calcDistance(self.curr_lat, self.curr_lon, self.dest_lat, self.dest_lon)
        bear = self.calcBearing(self.curr_lat, self.curr_lon, self.dest_lat, self.dest_lon)
        # print(dist, bear)
        if dist < 50 and dist > 1:
            # La referecia de Heading es el yaw actual + el bearing al WP
            y_ref = bear# + self.curr_y
            # y_ref = -y_ref
            print(y_ref, self.curr_y)
            # En función de la distancia hardcodeo 3 velocidades.
            fvel_ref = 0
            if dist > 2:
                fvel_ref = 0.3
            if dist > 6:
                fvel_ref = 0.5
            if dist > 10:
                fvel_ref = 1.0
        else :
            fvel_ref = 0
            y_ref = self.curr_y

        # Armo el mensaje a publicar
        message = CourseStamped()
        message.header.stamp = rospy.Time.now()
        message.speed = fvel_ref
        message.yaw = y_ref
        # Publico el mensaje
        self.reference_publisher.publish(message)

    #Haversine Formuala to find distance
    def calcDistance(self, lat1, lon1, lat2, lon2):
        lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        return 6371000 * c

    #Horisontal Bearing
    def calcBearing(self, lat1, lon1, lat2, lon2):
        dLon = lon2 - lon1
        y = sin(dLon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon)
        return atan2(y, x)

    def shutdown(self):
        """Unregisters publishers and subscribers and shutdowns timers"""
        self.publish_timer.shutdown()
        self.reference_publisher.unregister()
        self.curr_pos_sub.unregister()
        self.curr_pos_sub.unregister()
        rospy.loginfo("[GUID] Sayonara generador de referencia. Nos vemo' en Disney.")

def main():
    """Entrypoint del nodo"""
    rospy.init_node('guidance_gps', anonymous=True, log_level=rospy.INFO)
    node = GuidanceGPSNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[GUID] Received Keyboard Interrupt (^C). Shutting down.")

    node.shutdown()

if __name__ == '__main__':
    main()