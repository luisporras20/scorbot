#!/usr/bin/env python3
#################################################
# -------------------LIBRERIAS-------------------#
import sys
import rclpy
import select
import os
import numpy as np
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped  # Agregada esta línea
from math import radians, cos, sin, atan2, sqrt, degrees
import transforms3d.quaternions as tf_quaternions

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

rad = (3.1416 / 180)  # incrementaremos las posiciones grado a grado

# mensajes
msg = """
Ingresa las posiciones que tú quieras pa:
"""

e = """
Error
"""

# -----------------------------------------------#
#################################################

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher_node')
        self.trajectory_publisher = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.tf_publisher = self.create_publisher(TFMessage, "/tf", 10)
        self.joints = ['tubo1largo1', 'tubo1tubo2', 'largo2tubo3', 'tubo3tubo4']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]

# --Funcion para obtener la tecla presionada--#
def getkey(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#############################################

# Llamo la funcion principal
def main(args=None):
    ##----NO MODIFICAR----##
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    ##----NO MODIFICAR----##

    rclpy.init(args=args)
    joint = [0.0, 0.0, 0.0, 0.0]  # Posiciones iniciales de los joints #

    auxiliar = 0
    i = 1

    arm_trajectory = TrajectoryPublisher()

    try:
        print(msg)
        while(1):
            x = float(input("Digita x: "))
            y = float(input("Digita y: "))
            z = float(input("Digita z: "))
            thetax = float(input("Digite rotación en x: "))
            thetay = float(input("Digite rotación en y: "))
            thetaz = float(input("Digite rotación en z: "))
            rotation = [radians(thetax),
                radians(thetay),
                radians(thetaz)]
            position_t = np.array([x,
                           y,
                           z])

            scale_perception = np.array([0, 0, 0, 1])
            Rx = np.array([[1, 0, 0],
                   [0, cos(rotation[0]), -sin(rotation[0])],
                   [0, sin(rotation[0]), cos(rotation[0])]])

            Ry = np.array([[cos(rotation[1]), 0, sin(rotation[1])],
                   [0, 1, 0],
                   [-sin(rotation[1]), 0, cos(rotation[1])]])

            Rz = np.array([[cos(rotation[2]), -sin(rotation[2]), 0],
                   [sin(rotation[2]), cos(rotation[2]), 0],
                   [0, 0, 1]])
            

            key = getkey(settings)

            joint[0] = theta1
            joint[1] = theta2
            joint[2] = d3
            joint[3] = theta4
            print(theta1)
            print(theta2)
            print(d3)
            print(theta4)

            ########################################################
            # ------------------Enviar trayectoria------------------#
            arm_trajectory.joint_positions = [joint[0], joint[1], joint[2], joint[3]]

            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = arm_trajectory.joints

            point = JointTrajectoryPoint()
            point.positions = arm_trajectory.joint_positions
            point.time_from_start.sec = 1

            trajectory_msg.points.append(point)
            arm_trajectory.trajectory_publisher.publish(trajectory_msg)
            # ------------------Enviar trayectoria------------------#
            ########################################################

    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
