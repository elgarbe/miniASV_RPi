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
from geometry_msgs.msg import TwistStamped, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

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
        self.rate = float(rospy.get_param('~rate', '50'))    # 50 Hz
        self.logdir = rospy.get_param('~logdir', '/tmp')

        velocity_topic = rospy.get_param('~velocity_topic', None)
        reference_topic = rospy.get_param('~reference_topic', None)
        cmd_pwm_topic = rospy.get_param('~command_motor_topic', None)

        fecha = date.asctime( date.localtime(date.time()) ).replace(':','-').replace(' ', '_')
        path = '%s/%s' % (self.logdir,fecha)
        if not os.path.exists(path):
            os.makedirs(path)
        self.ident_file = open('%s/ident-accel.log' % path,'w')

        ########################################################################
        # Suscriptores
        ########################################################################
        # Me suscribo al tópico donde se envian los datos de velocidad
        self.velocity_subs = rospy.Subscriber(velocity_topic, Odometry, self.update)
        rospy.loginfo('[IDNT] Subscribing to ASV velocity topic: %s', velocity_topic)

        # En éste tópico le informo que ya puede empezar a generar la referencia deseada
        self.Start = rospy.Subscriber('/start', Float32, self.start)
        self.auto_ref = False
        self.log = False
        self.t_0 = 0
        self.pwm_l_step = 1500
        self.pwm_r_step = 1500
        self.pwm_l_r = 1500
        self.pwm_r_r = 1500

        # Creo el Publisher para los comandos de PWM a los motores
        self.pwm_publisher = rospy.Publisher(cmd_pwm_topic, Vector3, queue_size=10) # cluster message
        rospy.loginfo('[IDNT] Will publish PWM command to topic: %s', cmd_pwm_topic)

        # Creo el timer con el que lo voy a invocar
        self.publish_timer = rospy.Timer(rospy.Duration(1/self.rate), self.publish, oneshot=False)
        rospy.loginfo('[IDNT] Update rate: %f Hz', self.rate)

        rospy.loginfo("[IDNT] Start Identification Node ...")

    def update(self, velocity_msg):
        """ """
        if self.log:
            # Obtengo las mediciones de Forward Vel y Yaw Rate
            fvel_meas = velocity_msg.twist.twist.linear.x #np.sqrt(velocity_msg.twist.twist.linear.x**2 + velocity_msg.twist.twist.linear.y**2)
            yr_meas   = velocity_msg.twist.twist.angular.z
            # Armo un array con los datos a loguear
            datos = [fvel_meas, yr_meas, self.pwm_l_r, self.pwm_r_r]
            # Escribo en el archivo CSV
            time = velocity_msg.header.stamp.to_sec()
            self.ident_file.write(str(time)+str('\t'))
            self.ident_file.write('\t'.join( [str(e) for e in list(datos)] ))
            self.ident_file.write('\n')

    # Cuando punlico en /start tomo el dato del mensaje y comienzo a publicar
    # y logear el experimento
    def start(self, x):
        rospy.loginfo("[IDNT] Comenzando auto reference")
        self.t_0 = rospy.get_time()
        self.pwm_l_step = x.data
        self.pwm_r_step = x.data
        self.auto_ref = True

    def publish(self, event):
        """Docstring"""
        t = rospy.get_time() - self.t_0
        # Logueo durante 1 segundo la velocidad actual
        if self.auto_ref and t <= 1:
            self.log = True
	        rospy.loginfo("[IDNT] Inicio Experimento.")	
        # Por 15 segundos hago un escalon de velocidad
        elif self.auto_ref and t > 1 and t <= 16:
            self.pwm_l_r = self.pwm_l_step
            self.pwm_r_r = self.pwm_r_step
        # Durante 10 segundos registro la desaceleración
        elif self.auto_ref and t > 16 and t <= 26:
            self.pwm_l_step = 1500
            self.pwm_r_step = 1500
            self.pwm_l_r = 1500
            self.pwm_r_r = 1500
        # Fin del experimento
	    elif self.auto_ref and t > 26:
            self.auto_ref = 0
            self.log = False
	        rospy.loginfo("[IDNT] Fin Experimento.")	

        # Publico el comando de PWM a los motores para la Chori
        message = Vector3()
        message.x = self.pwm_l_r
        message.y = self.pwm_r_r
        self.pwm_publisher.publish(message)

    def shutdown(self):
        """Unregisters publishers and subscribers and shutdowns timers"""
        self.publish_timer.shutdown()
        self.velocity_subs.unregister()
        self.ident_file.close()
        self.pwm_publisher.unregister()

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