#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 30 18:52:30 2021 -0300

@author: elgarbe

Identificación de modelo dinámico del miniASV
"""
# NumPy
import numpy as np
import time as date
import os as os

# ROS Python API
import rospy
import message_filters

# Import the messages we're interested in sending and receiving, and having and sharing
from geometry_msgs.msg import TwistStamped, WrenchStamped, Vector3Stamped
from nav_msgs.msg import Odometry

MSG_QUEUE_MAXLEN = 10

class IdentWriterNode(object):
    """Nodo para realizar los experimentos de identificacion
    de modelo dinámico de ASV y registrar los datos en archivos CSV.

    Lee la velocidad y referencia en 2 tópicos distintos.
     * odometry/filtered: mensajes de tipo Odometry. La salida de robot_localization
     * chori/reference: mensajes de tipo Twist_Stamped, publicado por la Chori en funcion del RadioControl

    Utilizando el sincronizador automático de mensajes de ROS, cuando
    llegan 2 mensajes juntos de velocidad y referencia, se
    actualiza el control.

    """
    def __init__(self):
        self.rate = float(rospy.get_param('~rate', '100'))    # 100 Hz
        self.slop = float(rospy.get_param('~slop', 0.2))
        self.logdir = rospy.get_param('~logdir', '/tmp')

        velocity_topic = rospy.get_param('~velocity_topic', None)
        reference_topic = rospy.get_param('~reference_topic', None)
        cmd_wrench_topic = rospy.get_param('~command_motor_topic', None)

        fecha = date.asctime( date.localtime(date.time()) ).replace(':','-').replace(' ', '_')
        path = '%s/%s' % (self.logdir,fecha)
        if not os.path.exists(path):
            os.makedirs(path)
        self.ident_file = open('%s/identificacion.log' % path,'w')

        ########################################################################
        # Suscriptores
        ########################################################################
        # Me suscribo al tópico donde se envian los datos de velocidad
        self.velocity_subs = message_filters.Subscriber(velocity_topic, Odometry)
        rospy.loginfo('[IDNT] Subscribing to ASV velocity topic: %s', velocity_topic)
        # Me suscribo al tópico donde se envía la referencia
        self.reference_subs = message_filters.Subscriber(reference_topic, TwistStamped)
        rospy.loginfo('[IDNT] Subscribing to ASV reference topic: %s', reference_topic)
        # Me suscribo al tópico donde se envían los comando Fx y Tz a los motores
        self.cmd_wrench_subs = message_filters.Subscriber(cmd_wrench_topic, WrenchStamped)
        rospy.loginfo('[IDNT] Subscribing to ASV wrench command topic: %s', cmd_wrench_topic)
        # Me suscribo al tópico donde se envían los comando de PWM a los motores
        # self.cmd_pwm_subs = message_filters.Subscriber(reference_topic, TwistStamped)
        # rospy.loginfo('[PID] Subscribing to ASV reference topic: %s', reference_topic)

        # Sincronizador de mensajes...
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.velocity_subs, self.reference_subs, self.cmd_wrench_subs],
            queue_size=MSG_QUEUE_MAXLEN,
            slop=self.slop)
        self.synchronizer.registerCallback(self.update)

        rospy.loginfo("[IDNT] Start Identification Node ...")

    def update(self, velocity_msg, reference_msg, wrench_msg):
        """ """

        # Obtengo las mediciones de Forward Vel y Yaw Rate
        fvel_meas = velocity_msg.twist.twist.linear.x
        yr_meas   = velocity_msg.twist.twist.angular.z
        # Obtengo las referencias de Forward Vel y Yaw Rate
        fvel_ref = reference_msg.twist.linear.x
        yr_ref   = reference_msg.twist.angular.z
        # Obtengo los comandos de Fx y Tz
        fx = wrench_msg.force.x
        tz = wrench_msg.torque.z

        time = velocity_msg.header.stamp.to_sec()
        self.ident_file.write(str(time)+str(' '))
        self.ident_file.write('\t'.join([fvel_meas, yr_meas, fvel_ref, yr_ref, fx, tz]))
        self.ident_file.write('\n')




    def shutdown(self):
        """Unregisters publishers and subscribers and shutdowns timers"""
        self.velocity_subs.unregister()
        self.reference_subs.unregister()
        self.cmd_wrench_subs.unregister()
        self.ident_file.close()

        rospy.loginfo("[IDNT] Sayonara generador de referencia. Nos vemo' en Disney.")

def main():
    """Entrypoint del nodo"""
    rospy.init_node('ident_writer', anonymous=True, log_level=rospy.INFO)
    node = IdentWriterNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[IDNT] Received Keyboard Interrupt (^C). Shutting down.")

    node.shutdown()

if __name__ == '__main__':
    main()