#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point 
from std_srvs.srv import Empty 
import math

class TurtleControllerNode(Node): #definicao classe TurtleControllerNode

    def __init__(self):
        super().__init__("turtle_controller_test_vii")
        #self.previous_x_ = 0
        #self.previous_y_ = 0
        self.cmd_vel_publisher_= self.create_publisher(Twist, "/turtle1/cmd_vel",57) #publicar comandos velocidadde

        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback,57) #subscribe no tópico da posicao
        self.position_subscriber_ = self.create_subscription(Point, "/desired_position", self.position_callback, 57) #subscribe no tópico de posiçao desejada 
        self.v_subscriber_ = self.create_subscription (Twist, "/desired_vnom", self.v_callback, 10)
        self.w_subscriber_ = self.create_subscription(Twist, "/desired_wnom", self.w_callback, 10)

        self.reset_client = self.create_client(Empty, "/reset") #cliente para o serviço de reset da tartaruga
        self.reset_service = self.create_service(Empty, "/reset_turtle", self.reset_callback) #serviço personalizado /reset_turtle

        self.get_logger().info("Turtle controller has been started.") #info inicializacao
        self.vnom = 0.2 #vel linear nominal
        self.wnom = 0.8 #vel angular nom
        self.k_p = 1/0.2 #ganho proporcional para pos
        self.k_theta = self.wnom*2/math.pi #ganho proporcional para angulo

        self.x_d = 4.0 
        self.y_d = 4.0 

    def position_callback(self, msg: Point): #definir posicao desejada
        self.x_d = msg.x 
        self.y_d =msg.y 
        self.get_logger().info(f'Posição desejada: x={self.x_d}, y={self.y_d}')

    def v_callback(self, msg: Twist): #definir vnom
        self.vnom = msg.linear.x
        self.k_theta = self.wnom *2/ math.pi
        self.get_logger().info(f"Vel atualizada: vnom={self.vnom}")
    
    def w_callback(self, msg: Twist): #definir wnom
        self.wnom = msg.angular.z
        self.k_theta = self.wnom *2/ math.pi
        self.get_logger().info(f"Vel angular atualizada: wnom={self.wnom}")

    def reset_callback(self, request, response): #Callback para /reset_turtle 
        if not self.reset_client.wait_for_service(timeout_sec=2.0): # Verificar se o serviço /reset está disponível
            self.get_logger().error("Serviço /reset não está disponível!")
            return response
        
        #Chamar o serviço /reset para dar reset
        future = self.reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)

        #Mensagem de confirmação do reset
        self.get_logger().info("Reset da tartaruga realizado.")
        return response


#Funcao callback chamada sempre que recebe nova posicao
    def pose_callback(self, pose: Pose): 
        cmd = Twist() #Comando de vel a ser enviado
#        x_d = 4.0 #x desejado
#        y_d = 4.0 #y desejado
        
        #erro posicao
        error_x = self.x_d - pose.x
        error_y = self.y_d - pose.y
        error_pose = math.sqrt(error_x**2 + error_y**2)



        #condiçao erro pequeno
        if error_pose < 0.05:
            vel_d = 0.0
            ang_vel_d = 0.0
            self.get_logger().info("Trajectory completed :)")
        
        else:
            #erro angular
            theta_d = math.atan2(self.y_d - pose.y, self.x_d - pose.x)


            def normalizacao_angulo(angle):
                if angle >=0:
                    angle = math.fmod( angle + math.pi, 2*math.pi)
                    return angle - math.pi
                else:
                    angle = math.fmod( -angle + math.pi, 2*math.pi)
                    return -(angle - math.pi)
            def dif_angulos(theta_d, theta_atual):
                return normalizacao_angulo(normalizacao_angulo(theta_d) - normalizacao_angulo(theta_atual))
            
            error_theta = dif_angulos(theta_d, pose.theta)



            
            cmd.linear.x = 0.0
            if abs(error_theta) > math.pi/2:
                vel_d=0
                ang_vel_d= self.wnom* math.copysign(1.0,error_theta)
            
            elif abs(error_theta) <= math.pi/2: #igual a else
                ang_vel_d= self.wnom*error_theta*self.k_theta
                vel_d= self.vnom*math.cos(error_theta)*min(1,self.k_p*error_pose)
                
    
        cmd.linear.x = float(vel_d)
        cmd.angular.z = float(ang_vel_d)

        self.cmd_vel_publisher_.publish(cmd) #publicar comando vel


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

#if __name__ == '__main__': #*
#    main() #*

#posição desejada:
#ros2 topic pub /desired_position geometry_msgs/Point "{x: 0.0, y: 0.0, z: 0.0}"


#velocidade linear desejada:
#ros2 topic pub /desired_vnom geometry_msgs/Twist "{linear: {x: 0.4}}"

#velocidade angular deseada:
#ros2 topic pub /desired_wnom geometry_msgs/Twist "{angular: {z: 1.2}}"

#reset da posição:
#ros2 service call /reset_turtle std_srvs/srv/Empty