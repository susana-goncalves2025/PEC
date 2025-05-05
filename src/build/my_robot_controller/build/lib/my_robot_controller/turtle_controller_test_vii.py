#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class TurtleControllerNode(Node): #definicao classe TurtleControllerNode

    def __init__(self):
        super().__init__("turtle_controller_test_vii")
        #self.previous_x_ = 0
        #self.previous_y_ = 0
        self.cmd_vel_publisher_= self.create_publisher(Twist, "/turtle1/cmd_vel", 10) #publicar comandos velocidadde
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10) #subscribe no tópico da posicao
        self.get_logger().info("Turtle controller has been started.") #info inicializacao
        self.vnom = 1 #vel linear nominal
        self.wnom = 1 #vel angular nom
        self.k_p = 1/0.2 #ganho proporcional para pos
        self. k_theta = self.ang_vel_nom*2/math.pi #ganho proporcional para angulo


#Funcao callback chamada sempre que recebe nova posicao
    def pose_callback(self, pose: Pose): 
        cmd = Twist() #Comando de vel a ser enviado
        x_d = 0.0 #x desejado
        y_d = 0.0 #y desejado
        
        #erro posicao
        error_x = x_d - pose.x
        error_y = y_d - pose.y
        error_pose = math.sqrt(error_x**2 + error_y**2)



        #condiçao erro pequeno
        if error_pose < 0.01:
            cmd.linear.x = 0
            cmd.linear.y = 0
            cmd.angular.z = 0
            self.get_logger().info("Trajectory completed :)")
        
        else:
            #erro angular
            theta_d = math.atan2(y_d - pose.y, x_d - pose.x)
            error_theta = theta_d - pose.theta


            while error_theta > 0.1:
                cmd.linear.x = 0
                cmd.linear.y = 0
                if abs(error_theta) > math.pi/2:
                    ang_vel_d= self.wnom* math.copysign(error_theta)
                
                elif abs(error_theta) < math.pi/2:
                    ang_vel_d= self.wnom*error_theta*2/math.pi
            else:
                ang_vel_d=0
                vel_d= self.vnom*math.cos(error_theta)*min(1,self.k_p*error_pose)
                
    
            cmd.linear.x = vel_d * math.cos(theta_d)
            cmd.linear.y = vel_d * math.sin(theta_d)
            cmd.angular.z = ang_vel_d

        self.cmd_vel_publisher_.publish(cmd) #publicar comando vel


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

#vars a definir: pos final e vref (wnom, vnom)
#raciocinio: posatual dif pos des: calcular erro angular e virar, se erro angular =0 v linear é ...