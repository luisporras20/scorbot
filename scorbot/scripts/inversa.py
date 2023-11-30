#!/usr/bin/env python3
#################################################
#-------------------LIBRERIAS-------------------#
import sys, rclpy, select, os
import numpy as np
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
import math
           
if os.name == 'nt':
    import msvcrt
else:
    import termios, tty

rad=(3.1416/180) #incrementaremos las posiciones grado a grado

#mensajes
msg = """
Ingresa las posiciones que tú quieras pa: 

"""

e = """
Error
"""
#-----------------------------------------------#
#################################################

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        self.trajectory_publihser = self.create_publisher(JointTrajectory,"/joint_trajectory_controller/joint_trajectory", 10)
        #timer_period = 1
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['tubo1largo1','tubo1tubo2','largo2tubo3','tubo3tubo4']
        self.joint_positions = [0.0,0.0,0.0,0.0] 

        

#--Funcion para obtener la tecla presionada--#
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
   
#Llamo la funcion principal
def main(args=None):

    ##----NO MODIFICAR----##	
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    ##----NO MODIFICAR----##
    
    rclpy.init(args=args)
    joint = [0.0,0.0,0.0,0.0] # Posiciones iniciales de los joints #
           
    auxiliar = 0
    i = 1

    arm_trajectory = TrajectoryPublisher()  
      
    try:
        print(msg)
        while(1):
            x = float(input("Digita x: "))
            y=float(input("Digita y: "))
            z=float(input("Digita z: "))
            theta=float(input("Digite rotación en z: "))
            r12=math.pow(x,2)+math.pow(y,2)
            r1=math.sqrt(r12)
            thetadep2=(r12-299650)/293250
            while(thetadep2>1 or thetadep2<-1):

                print("La pose deseada está fuera del espacio de trabajo")
                x=float(input("Digite una distancia válida de la distancia en x: "))
                y=float(input("Digite una distancia válida de la distancia en y: "))
                z=float(input("Digite una distancia válida de la distancia en z: "))
                theta=float(input("Digite una rotación válida respecto a z: "))
                r12=math.pow(x,2)+math.pow(y,2)
                r1=math.sqrt(r12)
                thetadep2=(r12-299650)/293250

            theta2=math.acos(thetadep2)
            beta=math.atan2(y,x)
            ipsilon=math.acos((r12+61600)/(850*r1))

            if(theta2<=0):
                theta1=beta+ipsilon

            else:
                theta1=beta-ipsilon

            theta4=-theta1-theta2+theta*math.pi/180
            d3=(490-z)/1000 #Pa pasar eso a milimetros

            if(theta1>1.5708 or theta1<-1.5708 or theta4>1.5708 or theta4<-1.5708):
                theta2=-theta2
            
            if(theta2<=0):
                theta1=beta+ipsilon

            else:
                theta1=beta-ipsilon

            theta4=theta1+theta2-theta*math.pi/180

          

            if(theta1>1.5708 or theta1<-1.5708 or theta2>1.5708 or theta2<-1.5708 or d3>0.1 or d3<0 or theta4>1.5708 or theta4<-1.5708):
                if(theta1>1.5708):
                    print("El movimiento no es posible, por el ángulo theta1")
                    theta1 = 1.5708
                if(theta1<-1.5708):
                    print("El movimiento no es posible, por el ángulo theta1")
                    theta1 = -1.5708
                if(theta2>1.5708):
                    print("El movimiento no es posible, por el ángulo theta2")
                    theta2 = 1.5708
                if(theta2<-1.5708):
                    print("El movimiento no es posible, por el ángulo theta2")
                    theta2= -1.5708
                if(d3>0.1):
                    print("El movimiento no es posible, por la distancia d3")
                    d3 = 0.1
                if(d3<0):
                    print("El movimiento no es posible, por la distancia d3")
                    d3 = 0.0
                if(theta4>1.5708):
                    print("El movimiento no es posible, por el ángulo theta4")
                    theta4 = 1.5708
                if(theta4<-1.5708):
                    print("El movimiento no es posible, por el ángulo theta4")
                    theta4= -1.5708
            
                """
            # Destroy the node explicitly
            arm_trajectory.destroy_node()          
            rclpy.shutdown()
            break 
            """

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
            #------------------Enviar trayectoria------------------#
            arm_trajectory.joint_positions=[joint[0],joint[1],joint[2],joint[3]]

            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = arm_trajectory.joints

            point = JointTrajectoryPoint()
            point.positions = arm_trajectory.joint_positions
            point.time_from_start.sec = 1

            trajectory_msg.points.append(point)
            arm_trajectory.trajectory_publihser.publish(trajectory_msg)

            #------------------Enviar trayectoria------------------#            
            ########################################################
    except Exception as e:
        print(e)  

if __name__ == '__main__':
    main()
