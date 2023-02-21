#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 30 18:52:30 2021 -0300

@author: elgarbe

Controlador PID para el miniASV
"""
# NumPy
import numpy as np

# ROS Python API
import rospy
import message_filters

# Import the messages we're interested in sending and receiving, and having and sharing
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from asv_chori.msg import CourseStamped
from std_msgs.msg import UInt8

# Libraries for data manipulation
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# from pid import PIDController
from gpsic.controladores import pid

MSG_QUEUE_MAXLEN = 10

#miniASV Constants
BOAT_WIDTH      = 0.4   #m
MAX_FWD_THRUST  = 35.0  #Newtons -> 3.5 Kgf medidos en la pileta
MAX_BCK_THRUST  = 20.0  #Newtons -> Estimado, no medido, se puede medir
MAX_FWD_VEL     = 2     #m/s    -> Verificar en el arroyo si llega a esta velocidad
MAX_BCK_VEL     = 0.5   #m/s
MAX_OUTPUT      = 1     #       -> Esto no sé bien que es. Creo
MAX_YAW_RATE    = 0.5   #rad/s

class PIDYAWControllerNode(object):
    """

    """
    def __init__(self):
        self.rate = float(rospy.get_param('~rate', '100'))    # 100 Hz
        self.slop = float(rospy.get_param('~slop', 0.2))
        y_pid_param = rospy.get_param('~y_pid', None)
        ms_topic = rospy.get_param('~ms_topic', '/radio_control/ms')

        velocity_topic = rospy.get_param('~velocity_topic', None)
        reference_topic = rospy.get_param('~course_ref_topic', None)
        command_topic = rospy.get_param('~yaw_pid_twist_topic', None)

        # Arranco suponiendo ModeSwitch en Manual
        self.MS = 1
        # Para detectar el cambio en el ModeSwitch
        self.prevMS = 1

        ########################################################################################
        # Espero por los nodos que publican las velocidades y las referencias y obtengo
        # la primer referencia y velocidad
        rospy.loginfo("[PID-Y] Waiting for reference initialization..")
        reference = rospy.wait_for_message(reference_topic, CourseStamped)
        rospy.loginfo("[PID-Y] Waiting for velocity initialization..")
        velocity = rospy.wait_for_message(velocity_topic, Odometry)

        self.t = velocity.header.stamp.to_sec()

        # Creo el controlador PID para el yaw
        self.y_pid = pid.PIDController(None, self.t, self.wrap2pi, **y_pid_param)

        # Me suscribo al tópico donde se envian los datos de velocidad
        self.velocity_subs = message_filters.Subscriber(velocity_topic, Odometry)
        rospy.loginfo('[PID-Y] Subscribing to ASV velocity topic: %s', velocity_topic)
        # Me suscribo al tópico donde se envía la referencia
        self.reference_subs = message_filters.Subscriber(reference_topic, CourseStamped)
        rospy.loginfo('[PID-Y] Subscribing to ASV reference topic: %s', reference_topic)
        # Me suscribo al tópico del ModeSwitch
        self.ms_subs = rospy.Subscriber(ms_topic, UInt8, self.ms_cb)
        rospy.loginfo('[PID] Subscribing to RC ModeSwitch topic: %s', ms_topic)

        # Creo el publicador que despacha los mensajes de referencia de Velocidad
        self.command_publisher = rospy.Publisher(command_topic, TwistStamped, queue_size=MSG_QUEUE_MAXLEN)
        rospy.loginfo('[PID-Y] Will publish ASV Twist Reference to topic: %s', command_topic)

        # Sincronizador de mensajes...
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.velocity_subs, self.reference_subs],
            queue_size=MSG_QUEUE_MAXLEN,
            slop=self.slop)
        self.synchronizer.registerCallback(self.update)

        rospy.loginfo("[PID-Y] Starting ASV PID Controller...")

    def wrap2pi(self, angle):
        """Return input angle wraped between [-np.pi, np.pi)"""
        return np.mod(angle + np.pi, 2 * np.pi) - np.pi

    def ms_cb(self, msg):
        self.MS = msg.data

    def update(self, velocity_msg, reference_msg):
        """Compute control action based on references and velocities messages"""

        # Obtengo las mediciones de Forward Vel, Yaw Rate y Yaw
        quat = (velocity_msg.pose.pose.orientation.x, velocity_msg.pose.pose.orientation.y, velocity_msg.pose.pose.orientation.z, velocity_msg.pose.pose.orientation.w)
        _, _, y_meas = euler_from_quaternion(quat)

        # Obtengo las referencias de Forward Vel y Yaw
        fvel_ref = reference_msg.speed
        y_ref  = reference_msg.yaw
            
        # Calculo el intervalo de tiempo entre las muestras
        delta_t = velocity_msg.header.stamp.to_sec() - self.t
        self.t = velocity_msg.header.stamp.to_sec()

        if delta_t > 0.0:
            # Verifico si se pasó de manual a automático
            if ((self.MS == 2) & (self.prevMS == 1)):
                rospy.loginfo("[PID-Y] Resetting Integral term...")
                # Cuando se produce el cambio de M a A reseteo el término integral
                # TODO: Otra forma sería resetearlo siempre que estemos en manual
                self.y_pid.i = np.zeros_like(self.y_pid.i)
                self.prevMS = 2
            if ((self.MS == 1) & (self.prevMS == 2)):
                self.prevMS = 1

            # Genero las acciones de control
            yr_ref = self.y_pid.update(y_ref, y_meas, self.t)
            # print ('yr: ', yr_ref, yr_meas, yr_cmd)

            # Creo el mensaje de referencia de Velocidad
            message = TwistStamped()
            message.header.stamp = rospy.Time.now()
            message.twist.linear.x = fvel_ref
            message.twist.angular.z = yr_ref
            self.command_publisher.publish(message)

    def shutdown(self):
        """Unregisters publishers and subscribers and shutdowns timers"""
        self.velocity_subs.unregister()
        self.reference_subs.unregister()
        self.cmd_motors_publisher.unregister()
        self.command_publisher.unregister()
        rospy.loginfo("[PID-Y] Sayonara YAW PID Controller. Nos vemo' en Disney.")

def main():
    """Entrypoint del nodo"""
    rospy.init_node('asv_pid_yaw', anonymous=True, log_level=rospy.INFO)
    node = PIDYAWControllerNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[PID-Y] Received Keyboard Interrupt (^C). Shutting down.")

    node.shutdown()

if __name__ == '__main__':
    main()