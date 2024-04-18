#!/usr/bin/env python
import rospy
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
import math
from math import sqrt
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportRelative
from turtlesim.msg import Pose
class MoveTurtlePIDControl:
    def __init__(self):
        rospy.init_node('controlAtg')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        #self.spawn_publisher = rospy.Publisher('/turtle1/spawn')
        rospy.wait_for_service('/spawn')
        rospy.wait_for_service('/kill')
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(20)

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y

    def posicion(self,xd,yd):
        x = self.current_x
        y = self.current_y 
        dtg = sqrt((xd-x)**2+(yd-y)**2)
        atg = math.atan2((yd-y),(xd-x))*180/(math.pi)
        info = [dtg, atg]
        return info
               
    def get_desired_x_from_user(self):
       print("Ingrese la posición deseada en el eje x:")
       return float(input("Coordenada x: "))

    def get_desired_y_from_user(self):
        print("Ingrese la posición deseada en el eje y:")
        return float(input("Coordenada y: "))

    def move_turtle_interactively(self):
        xd = self.get_desired_x_from_user()
        yd = self.get_desired_y_from_user()
        ap=self.posicion(xd,yd)
        dtg=float(ap[0])
        atg=float(ap[1]) 
        rospy.loginfo("DTG: %f  ATG: %f " , dtg, atg)
        spawn_client = rospy.ServiceProxy('/spawn', Spawn)
        kill_client = rospy.ServiceProxy('/kill', Kill)
        respk = kill_client(name='turtle1')
        resp = spawn_client(x=xd, y=yd, theta=atg*math.pi/180,name='turtle1')
       
if __name__ == '__main__':
    try:
        move_turtle_pid = MoveTurtlePIDControl()
        move_turtle_pid.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass


