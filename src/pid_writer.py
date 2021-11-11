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
from geometry_msgs.msg import TwistStamped, WrenchStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

MSG_QUEUE_MAXLEN = 10

class PIDWriterNode(object):
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
        self.rate = float(rospy.get_param('~rate', '50'))    # 50 Hz
        self.logdir = rospy.get_param('~logdir', '/tmp')
        self.slop = float(rospy.get_param('~slop', 0.2))
        fvel_pid_param = rospy.get_param('~fvel_pid', None)
        yr_pid_param = rospy.get_param('~yr_pid', None)

        velocity_topic = rospy.get_param('~velocity_topic', None)
        reference_topic = rospy.get_param('~twist_ref_topic', None)
        cmd_wrench_topic = rospy.get_param('~command_wrench_topic', None)

        fecha = date.asctime( date.localtime(date.time()) ).replace(':','-').replace(' ', '_')
        path = '%s/%s' % (self.logdir,fecha)
        filename = 'pid-fv-{kp}-{ki}-{kd}'.format(**fvel_pid_param)
        filename = filename + '-yr-{kp}-{ki}-{kd}'.format(**yr_pid_param)
        if not os.path.exists(path):
            os.makedirs(path)
        self.pid_file = open('%s/%s.log' % (path, filename),'w')

        ########################################################################
        # Suscriptores
        ########################################################################
        # Me suscribo al tópico donde se envian los datos de velocidad
        self.vel_subs = message_filters.Subscriber(velocity_topic, Odometry)
        rospy.loginfo('[PID-WR] Subscribing to ASV velocity topic: %s', velocity_topic)
        # Me suscribo al tópico donde se envian la referencia
        self.ref_subs = message_filters.Subscriber(reference_topic, TwistStamped)
        rospy.loginfo('[PID-WR] Subscribing to Reference topic: %s', reference_topic)
        # Me suscribo al tópico donde se envian los comandos a los motores
        self.cmd_subs = message_filters.Subscriber(cmd_wrench_topic, WrenchStamped)
        rospy.loginfo('[PID-WR] Subscribing to ASV velocity topic: %s', cmd_wrench_topic)

        # Sincronizador de mensajes...
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.vel_subs, self.ref_subs, self.cmd_subs],
            queue_size=MSG_QUEUE_MAXLEN, slop=self.slop)
        self.synchronizer.registerCallback(self.update)

        rospy.loginfo("[PID-WR] Start Logging Data ...")

    def update(self, vel_msg, ref_msg, cmd_msg):
        """ """
        # Obtengo las mediciones de Forward Vel y Yaw Rate
        fvel_meas = vel_msg.twist.twist.linear.x
        yr_meas   = vel_msg.twist.twist.angular.z
        # Obtengo las referencias de Forward Vel y Yaw Rate
        fvel_ref = ref_msg.twist.linear.x
        yr_ref   = -ref_msg.twist.angular.z
        # Obtengo las referencias de Forward Vel y Yaw Rate
        fx = cmd_msg.wrench.force.x
        tz = cmd_msg.wrench.torque.z

        # Armo un array con los datos a loguear
        datos = [fvel_ref, yr_ref, fvel_meas, yr_meas, fx, tz]
        # Escribo en el archivo CSV
        time = vel_msg.header.stamp.to_sec()
        self.pid_file.write(str(time)+str('\t'))
        self.pid_file.write('\t'.join( [str(e) for e in list(datos)] ))
        self.pid_file.write('\n')


    def shutdown(self):
        """Unregisters publishers and subscribers and shutdowns timers"""
        self.vel_subs.unregister()
        self.ref_subs.unregister()
        self.cmd_subs.unregister()
        self.pid_file.close()

        rospy.loginfo("[PID-WR] Sayonara generador de referencia. Nos vemo' en Disney.")

def main():
    """Entrypoint del nodo"""
    rospy.init_node('pid_writer', anonymous=True, log_level=rospy.INFO)
    node = PIDWriterNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[PID-WR] Received Keyboard Interrupt (^C). Shutting down.")

    node.shutdown()

if __name__ == '__main__':
    main()