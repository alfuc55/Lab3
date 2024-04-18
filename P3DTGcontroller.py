import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, radians
import math
from math import sqrt
class MoveTurtlePIDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_x')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(20)
        
        self.current_x = 0
        self.current_y = 0
        self.current_t = 0
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_t = 0
        self.error_accumulation = 0
        self.q1anterior = 0
        

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_t = pose.theta
    def move_turtle_to_desired_x(self, desired_x):
        # Constantes de proporcionalidad, integral y derivativa del controlador (ajustables)
        Kp = 1
        Ki = 0.01
        Kd = 0.1
        x = self.current_x
        q1 = self.current_t
        while not rospy.is_shutdown():
            # Calcular el error de posición
            #Ajuste del valor actual de x con referencia a la tortuga 
                #dx*cos(q1) - conj(y)*sin(q1) - cos(q1)*conj(x) + dy*sin(q1)
            #currentx = self.current_x*math.cos(q1) - y*math.sin(q1) - math.cos(q1)*x + self.current_y*math.sin(q1)
            error_x = abs(desired_x - self.current_x)
            
            # Sumar el error a la acumulación de errores
            self.error_accumulation += error_x
            
            # Calcular la velocidad lineal del movimiento
            vel_x = Kp * error_x + Ki * self.error_accumulation + Kd * (error_x - self.last_error_x)
            
            # Guardar el error actual para usarlo en la próxima iteración
            self.last_error_x = error_x
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual, el error y la variable vel_x en la terminal
            rospy.loginfo("Posición actual: %f, Error: %f, Velocidad lineal: %f", self.current_x, error_x, vel_x)
            
            # Verificar si se alcanza la posición deseada
            if abs(error_x) < 0.1:
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()

    

    def move_turtle_to_desired_t(self, desired_t):
        # Constantes de proporcionalidad, integral y derivativa del controlador (ajustables)
        Kp = 1
        Ki = 0.00
        Kd = 0.0
        self.error_accumulation = 0 
        desiredt=desired_t
        while not rospy.is_shutdown():
            
            # Calcular el error de posición
            error_t = desiredt - self.current_t
            
            # Sumar el error a la acumulación de errores
            self.error_accumulation += error_t
            
            # Calcular la velocidad lineal del movimiento
            vel_t = Kp * error_t + Ki * self.error_accumulation + Kd * (error_t - self.last_error_t)
            
            # Guardar el error actual para usarlo en la próxima iteración
            self.last_error_t = error_t
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.angular.z = vel_t
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Imprimir la posición actual, el error y la variable vel_x en la terminal
            rospy.loginfo("Posición actual: %f, Error: %f, Velocidad angular: %f", self.current_t, error_t, vel_t)
            
            # Verificar si se alcanza la posición deseada
            if abs(error_t) < 0.05:
                rospy.loginfo("Posición deseada alcanzada")
               
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()
            self.q1anterior = self.current_t


    def get_desired_x_from_user(self):
        print("Ingrese la posición deseada en el eje x:")
        return float(input("Coordenada x: "))

    def get_desired_y_from_user(self):
        print("Ingrese la posición deseada en el eje y:")
        return float(input("Coordenada y: "))
    
    def get_desired_t_from_user(self):
        print("Ingrese la posición deseada de rotacion:")
        pt = float(input("angulo theta: "))*(math.pi/180)
        return pt
    def posicion(self,xd,yd):
        x = self.current_x
        y = self.current_y 
        dtg = sqrt((xd-x)**2+(yd-y)**2)
        atg = math.atan2((yd-y),(xd-x))
        info = [dtg, atg]
        return info
    
    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la posición deseada del usuario
            desired_x = self.get_desired_x_from_user()
            desired_y = self.get_desired_y_from_user()
            ap=self.posicion(desired_x,desired_y)
            dtg=float(ap[0])
            atg=float(ap[1])
            xa=self.current_x+dtg*math.cos(atg)
            ya=self.current_y+dtg*math.sin(atg)
            rospy.loginfo("DTG: %f  ATG: %f xa=%f ya%f" , dtg, atg*180/(math.pi),xa,ya)
            # Mover la tortuga a la posición deseada
            self.move_turtle_to_desired_t(atg)
            self.move_turtle_to_desired_x(xa)
           
            
if __name__ == '__main__':
    try:
        move_turtle_pid = MoveTurtlePIDControl()
        move_turtle_pid.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass