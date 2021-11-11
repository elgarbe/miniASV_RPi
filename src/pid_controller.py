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
from geometry_msgs.msg import TwistStamped, WrenchStamped, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from asv_chori.msg import CourseStamped

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


TWIST_CTRL = 0

class PIDControllerNode(object):
    """Este Controlador es para un cluster de 2 UAV.

    Lee la velocidad y referencia en 2 tópicos distintos.
     * odometry/filtered: mensajes de tipo Odometry. La salida de robot_localization
     * chori/reference: mensajes de tipo Twist_Stamped, publicado por la Chori en funcion del RadioControl

    Publica el comando en miniASV/command, de tipo WrenchStamped
    Publica el comando en cmd_pwm, de tipo Vector3, al cual se suscribe la chori

    Utilizando el sincronizador automático de mensajes de ROS, cuando
    llegan 2 mensajes juntos de velocidad y referencia, se
    actualiza el control.

    """
    def __init__(self):
        self.rate = float(rospy.get_param('~rate', '100'))    # 100 Hz
        self.slop = float(rospy.get_param('~slop', 0.2))
        fvel_pid_param = rospy.get_param('~fvel_pid', None)
        yr_pid_param = rospy.get_param('~yr_pid', None)
        y_pid_param = rospy.get_param('~y_pid', None)

        velocity_topic = rospy.get_param('~velocity_topic', None)
        if TWIST_CTRL:
            # Twist controller
            reference_topic = rospy.get_param('~twist_ref_topic', None)
            rospy.loginfo("[PID] TWIST control mode.")
        else:
            # Course controller
            reference_topic = rospy.get_param('~course_ref_topic', None)
            rospy.loginfo("[PID] COURSE control mode.")

        command_topic = rospy.get_param('~command_wrench_topic', None)
        command_motors_topic = rospy.get_param('~command_motor_topic', None)

        ########################################################################################
        # Espero por los nodos que publican las velocidades y las referencias y obtengo
        # la primer referencia y velocidad
        rospy.loginfo("[PID] Waiting for reference initialization..")
        if TWIST_CTRL:
            # Twist controller
            reference_topic = rospy.get_param('~twist_ref_topic', None)
            reference = rospy.wait_for_message(reference_topic, TwistStamped)
        else:
            # Course controller
            reference_topic = rospy.get_param('~course_ref_topic', None)
            reference = rospy.wait_for_message(reference_topic, CourseStamped)
        rospy.loginfo("[PID] Waiting for velocity initialization..")
        velocity = rospy.wait_for_message(velocity_topic, Odometry)

        self.t = velocity.header.stamp.to_sec()

        # TODO: Por ahora son 3 PID, quizás podría usar una array de ellos como en el cluster
        self.fvel_pid = pid.PIDController(None, self.t, None, **fvel_pid_param)
        self.yr_pid = pid.PIDController(None, self.t, None, **yr_pid_param)
        self.y_pid = pid.PIDController(None, self.t, self.wrap2pi, **y_pid_param)

        # self.fvel_pid = PIDController(kp=1.0, ki=0.5, kd=0.0, t0=self.t, isats=[-2.0, 2.0])
        # self.yr_pid = PIDController(kp=1.0, ki=0.5, kd=0.0, t0=self.t, isats=[-0.5, 0.5])

        # Me suscribo al tópico donde se envian los datos de velocidad
        self.velocity_subs = message_filters.Subscriber(velocity_topic, Odometry)
        rospy.loginfo('[PID] Subscribing to ASV velocity topic: %s', velocity_topic)

        # Me suscribo al tópico donde se envía la referencia
        if TWIST_CTRL:
            self.reference_subs = message_filters.Subscriber(reference_topic, TwistStamped)
        else:
            self.reference_subs = message_filters.Subscriber(reference_topic, CourseStamped)
        rospy.loginfo('[PID] Subscribing to ASV reference topic: %s', reference_topic)

        # Creo el publicador que despacha los mensajes de fuerzas/torques de control
        self.command_publisher = rospy.Publisher(command_topic, WrenchStamped, queue_size=MSG_QUEUE_MAXLEN)
        rospy.loginfo('[PID] Will publish ASV force/torque commands to topic: %s', command_topic)

        # Creo el publicador que despacha los mensajes del control a los motores
        self.cmd_motors_publisher = rospy.Publisher(command_motors_topic, Vector3, queue_size=MSG_QUEUE_MAXLEN)
        rospy.loginfo('[PID] Will publish ASV motor commands to topic: %s', command_motors_topic)

        # Sincronizador de mensajes...
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.velocity_subs, self.reference_subs],
            queue_size=MSG_QUEUE_MAXLEN,
            slop=self.slop)
        self.synchronizer.registerCallback(self.update)

        rospy.loginfo("[PID] Starting ASV PID Controller...")

    def wrap2pi(self, angle):
        """Return input angle wraped between [-np.pi, np.pi)"""
        return np.mod(angle + np.pi, 2 * np.pi) - np.pi

    def update(self, velocity_msg, reference_msg):
        """Compute control action based on references and velocities messages"""

        # Obtengo las mediciones de Forward Vel, Yaw Rate y Yaw
        fvel_meas = velocity_msg.twist.twist.linear.x
        yr_meas   = velocity_msg.twist.twist.angular.z

        quat = (velocity_msg.pose.pose.orientation.x, velocity_msg.pose.pose.orientation.y, velocity_msg.pose.pose.orientation.z, velocity_msg.pose.pose.orientation.w)
        _, _, y_meas = euler_from_quaternion(quat)

        # Obtengo las referencias de Forward Vel y Yaw Rate/Yaw
        if TWIST_CTRL:
            fvel_ref = reference_msg.twist.linear.x
            yr_ref = -reference_msg.twist.angular.z
        else:
            fvel_ref = reference_msg.speed
            yr_ref = 0
            y_ref  = reference_msg.yaw
            
        # Calculo el intervalo de tiempo entre las muestras
        delta_t = velocity_msg.header.stamp.to_sec() - self.t
        self.t = velocity_msg.header.stamp.to_sec()

        if delta_t > 0.0:
            # Genero las acciones de control
            fvel_cmd = self.fvel_pid.update(fvel_ref, fvel_meas, self.t)
            # fvel_cmd=0
            # print(fvel_ref, fvel_meas, self.t, fvel_cmd)

            if not TWIST_CTRL:
                yr_ref = self.y_pid.update(y_ref, y_meas, self.t)
                # print('yaw:', y_ref, y_meas, yr_ref)

            yr_cmd  = self.yr_pid.update(yr_ref, yr_meas, self.t)

            # print ('yr: ', yr_ref, yr_meas, yr_cmd)

            fx = fvel_cmd       #self.deadzone_force(fvel_cmd, 2*MAX_FWD_THRUST * 0.06, 2*MAX_BCK_THRUST * 0.06)
            tz = -yr_cmd         #self.deadzone_force(yr_cmd, 2, 2)
            # Creo el mensaje de las fuerzas y torques solicitados
            message = WrenchStamped()
            message.header.stamp = rospy.Time.now()
            message.wrench.force.x = fx
            message.wrench.torque.z = tz
            self.command_publisher.publish(message)

            # Con las fuerzas/torques hago la conversión a thrust the cada motor
            # y publico dicho comando
            ##############################################################
            # Conversion de Fuerza_X y Torque_Z en thrust de cada motor
            # Algoritmos usados en Heron
            # self.compensate_force(fx, tz)

            ##############################################################
            # Conversion de Fuerza_X y Torque_Z en thrust de cada motor
            left_thrust = fx/2 + tz/BOAT_WIDTH
            right_thrust = fx/2 - tz/BOAT_WIDTH
            # Preparo el mensaje de PWM (??) para cada motor
            cmd_output = Vector3()
            # Convierto Thrust en PWM
            cmd_output.x = self.calculate_motor_setting (left_thrust)
            cmd_output.y = self.calculate_motor_setting (right_thrust)
            self.cmd_motors_publisher.publish(cmd_output)

    def compensate_force(self, force_x, tau_z):
        fx = force_x
        tauz = tau_z
        # Maximo Torque posible alrededor de Z 
        # Se tiene en cuenta el Thrust en backward ya que es menor
        # al en forward
        max_tauz = MAX_BCK_THRUST*2*BOAT_WIDTH
        # Clampeo el torque del PID 
        tauz = np.clip(tauz, -max_tauz, max_tauz)

        # Thrust de cada motor para conseguir el Torque en Z del PID
        left_thrust = -tauz/(2*BOAT_WIDTH)
        right_thrust = tauz/(2*BOAT_WIDTH)

        # Calculamos el máximo Thrust para cumplir con la fuerza en X
        # pedida por el PID LUEGO de asegurar el thrust para el giro
        max_fx = 0
        if (tauz >= 0): 
            if (fx >= 0):   #forward thrust on the left thruster will be limiting factor
                max_fx = (MAX_FWD_THRUST - left_thrust) * 2
                fx = np.minimum(max_fx,fx)
            else :          #backward thrust on the right thruster will be limiting factor
                max_fx = (-MAX_BCK_THRUST - right_thrust) * 2
                fx = np.maximum(max_fx,fx)
        else :
            if (fx >= 0 ) :
                max_fx = (MAX_FWD_THRUST - right_thrust) * 2
                fx = np.minimum(max_fx,fx)
            else :
                max_fx = (-MAX_BCK_THRUST - left_thrust) * 2
                fx = np.maximum(max_fx,fx)

        # Sumo al thrust calculado el necesario para cumplir con la fuerza en X
        left_thrust += fx/2.0
        right_thrust += fx/2.0
        
        # Saturo los valores de thrust de cada motor
        left_thrust = self.saturate_thrusters(left_thrust)
        right_thrust = self.saturate_thrusters(right_thrust)

        # Preparo el mensaje de PWM (??) para cada motor
        cmd_output = Vector3()
        # Convierto Thrust en PWM
        cmd_output.x = self.calculate_motor_setting (left_thrust)
        cmd_output.y = self.calculate_motor_setting (right_thrust)
        self.cmd_motors_publisher.publish(cmd_output)

    # Función para asegurar una fuerza mínima entre 2 límites
    def deadzone_force(self, force, pos_limit, neg_limit): 
        if (force > 0) :
            if (force < pos_limit) :
                return 0
        else :
            if (force > -neg_limit) :
                return 0

        return force

    # Función para clampear el Trhust calculado entre los valores Máximos
    # y mínimos posibles de alcanzar
    def saturate_thrusters (self, thrust):
        thrust = np.minimum(MAX_FWD_THRUST,thrust)
        thrust = np.maximum(-1*MAX_BCK_THRUST,thrust)
        return thrust

    # Función para convertir Thrust en PWM. 
    # TODO: Debería salir de la curva relevada en la pileta:
    # Thrust = 0.009876 * PWM - 15.0854
    # PWM = (Thrust + 15.0854) / 0.009876 = Trhust * 101.25 + 1500
    # TODO: Ver en Backward
    def calculate_motor_setting (self, thrust):
        output = 1500.0
        if (thrust > 0):
            output = thrust * 101.25 + 1500 #(MAX_OUTPUT/MAX_FWD_THRUST)
        else:
            if (thrust < 0):
                output = thrust * 101.25 + 1500 #(MAX_OUTPUT/MAX_BCK_THRUST)
        return output

    def shutdown(self):
        """Unregisters publishers and subscribers and shutdowns timers"""
        self.velocity_subs.unregister()
        self.reference_subs.unregister()
        self.cmd_motors_publisher.unregister()
        self.command_publisher.unregister()
        rospy.loginfo("[PID] Sayonara generador de referencia. Nos vemo' en Disney.")

def main():
    """Entrypoint del nodo"""
    rospy.init_node('asv_pid', anonymous=True, log_level=rospy.INFO)
    node = PIDControllerNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[PID] Received Keyboard Interrupt (^C). Shutting down.")

    node.shutdown()

if __name__ == '__main__':
    main()