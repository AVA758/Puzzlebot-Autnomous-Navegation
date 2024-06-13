import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_bueno')
        self.bridge = CvBridge()

        # SUSCRIPTORES
        self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10) #Video
        self.sub2 = self.create_subscription(String, '/sign_order', self.sign_callback, 10) #Señalamiento
        self.sub3 = self.create_subscription(String, '/traffic_color', self.traffic_callback, 10) #Semáforo

        # PUBLICADORES
        self.pub = self.create_publisher(Image, 'processed_img', 10) #Video
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10) #Velocidad
        
        # DECLARACIÓN DE PARÁMETROS
        self.robot_vel = Twist()
        self.image_received_flag = False
        self.intersect_flag = False
        self.traffic_received = "NOSEM"
        self.left_flag = False

        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info('Line Follower Node started')

        self.flag=False
        self.distance = 0.0
        self.state = "stop"
        self.next = True
        self.time = 0.0
        self.v = 0.0 #[m/s] 

        self.stop_start_time = None
        self.progress_start_time = None

        self.order_received = "Non"
        self.last_order_received = "Non"
        


    # DETECCIÓN DE SEÑALAMIENTO

    def sign_callback(self, msg):
        if msg.data != self.last_order_received:  # Check if the new message is different
            self.order_received = msg.data
            self.last_order_received = msg.data  # Update the last received order


    # DETECCIÓN DE COLOR DE SEMÁFORO
    def traffic_callback(self, msg):
        self.traffic_received = msg.data


    # IMPORTACIÓN DE VIDEO DE PUZZLEBOT
    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except Exception as e:
            self.get_logger().info('Failed to get an image: {}'.format(str(e)))


    # SECUENCIA PRINCIPAL
    def timer_callback(self):
        if self.image_received_flag:
            # AJUSTES DE IMAGEN Y CONTORNOS
            image = self.cv_img.copy()
            region_of_interest = image[161:240, 0:260]
            intersection_img = image[120:240, 173:260]
            intersection_img = cv2.cvtColor(intersection_img, cv2.COLOR_BGR2GRAY)
            intersection_img = cv2.cvtColor(intersection_img, cv2.COLOR_GRAY2BGR)
            hsv = cv2.cvtColor(intersection_img, cv2.COLOR_BGR2HSV)
            min_b = np.array([0, 0, 0])
            max_b = np.array([0, 0, 80])
            mask_b = cv2.inRange(hsv, min_b, max_b)
            mask_b = cv2.erode(mask_b, None, iterations=2)
            mask_b = cv2.dilate(mask_b, None, iterations=2)
            res_b = cv2.bitwise_and(intersection_img, intersection_img, mask=mask_b)
            contours2, hierarchy = cv2.findContours(mask_b.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            blurred = cv2.GaussianBlur(image, (5, 5), 0)
            gray = cv2.cvtColor(region_of_interest, cv2.COLOR_BGR2GRAY)
            _, thresholded = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours and len(contours2) <= 2 and not self.intersect_flag:
                self.intersect_flag = False
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                self.flag= False
                if len(contours2) > 2:
                    self.get_logger().info('INTERSECCIÓN dentro')
                    self.robot_vel.angular.z = 0.0
                    self.robot_vel.linear.x = 0.0
                    self.intersect_flag = True
                    self.flag= True 

                elif M['m00'] != 0:
                    self.get_logger().info('LINEA DETECTADA')
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(region_of_interest, (cx, cy), 10, (0, 255, 0), -1)
                    error = cx - image.shape[1] // 2
                    self.robot_vel.angular.z = -float(error) / 100


                    self.robot_vel.linear.x = 0.1
                    


                    if self.traffic_received == "verde":
                        self.get_logger().info('LINEA DETECTADA SEMÁFORO VERDE A LA VISTA')
                        self.robot_vel.linear.x = 0.1
                    elif self.traffic_received == "amarillo":
                        self.get_logger().info('LINEA DETECTADA SEMÁFORO AMARILLO A LA VISTA')
                        self.robot_vel.linear.x = 0.05
                

               
                else:
                    self.robot_vel.angular.z = 0.0
                    self.robot_vel.linear.x = 0.0



            elif len(contours2) > 2 and not self.flag and self.next:
                self.get_logger().info('INTERSECCIÓN')
                self.robot_vel.angular.z = 0.0
                self.robot_vel.linear.x = 0.0
                self.intersect_flag = True
                self.flag= True



            #######################################################################################################
            #      SEÑAL DE CONTINÚA AL FRENTE
            #######################################################################################################
            if self.order_received == "frente" and self.intersect_flag== True:
                self.get_logger().info('FRENTE')
                self.next= False
                self.distance = 0.6
                self.time = 4.0
                self.flag= False

                if self.state == "stop":
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0
                    self.t_linear = int(self.time * 10**9)
                    self.v_linear = self.distance / self.time
                    self.cmd_vel_pub.publish(self.robot_vel)
                    #self.state = "move_linear"

                    if self.traffic_received == "verde" or self.traffic_received == "amarillo":
                        self.state = "move_linear"
                        self.start_time = self.get_clock().now().nanoseconds
                        
                    elif self.traffic_received == "rojo":
                        self.state = "stop"
                    
                
                elif self.state == "move_linear":   
                    current_time = self.get_clock().now().nanoseconds
                    self.robot_vel.linear.x = self.v_linear
                    self.robot_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(self.robot_vel)
                    if current_time - self.start_time >= self.t_linear:
                        self.start_time = self.get_clock().now().nanoseconds
                        self.intersect_flag = False
                        self.order_received = "Non"
                        self.traffic_received = "NOSEM"
                        self.state = "stop"
                        self.next= True
                    
                   

                

            #######################################################################################################
            #      SEÑAL DE GIRO A LA IZQUIERDA
            #######################################################################################################
            elif self.order_received == "left" and self.intersect_flag== True:
                self.left_flag = True
                self.get_logger().info('Vuelta a la izquierda')
                self.next = False
                self.distance = 0.35
                self.time = 3.0


                self.time_extra=1.0
                self.distance_extra = 0.1

                self.flag= False
                
                if self.state == "stop":
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0
                    self.t_linear = int(self.time * 10**9)
                    self.v_linear = self.distance / self.time

                    self.t_angular = int(self.time * 10**9)
                    self.v_angular = (np.pi/2) / (self.t_angular*10**-9)

                    self.t_extra = int(self.time_extra * 10**9)
                    self.v_extra = self.distance_extra / self.time_extra

                    self.cmd_vel_pub.publish(self.robot_vel)
                    #self.state = "move_left"

                    if self.traffic_received == "verde" or self.traffic_received == "amarillo":
                        self.state = "move_left"
                        self.start_time = self.get_clock().now().nanoseconds
                    
                    elif self.traffic_received == "rojo":
                        self.state = "stop"
                    
                
                elif self.state == "move_left":

                    current_time = self.get_clock().now().nanoseconds
                    if current_time - self.start_time <= self.t_linear:
                        self.robot_vel.linear.x = self.v_linear
                        self.robot_vel.angular.z = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)
                    else:
                        if current_time - self.start_time <= self.t_linear + self.t_angular:
                            self.get_logger().info('Vuelta')
                            self.robot_vel.linear.x = 0.0 
                            self.robot_vel.angular.z = self.v_angular

                            self.cmd_vel_pub.publish(self.robot_vel)      

                        else:
                            if current_time - self.start_time <= self.t_linear + self.t_angular + self.t_extra:
                                self.get_logger().info('Alineando')
                                self.robot_vel.linear.x = self.v_linear
                                self.robot_vel.angular.z = 0.0 
                                
                                self.cmd_vel_pub.publish(self.robot_vel)
                            
                            else:
                                self.start_time = self.get_clock().now().nanoseconds
                                self.intersect_flag = False
                                self.order_received = "Non"
                                self.traffic_received = "NOSEM"
                                self.state = "stop"
                                self.next= True
                                self.left_flag = False
                    self.cmd_vel_pub.publish(self.robot_vel)
                        
              

            
            #######################################################################################################
            #      SEÑAL DE GIRO A LA DERECHA
            #######################################################################################################
            elif self.order_received == "right" and self.intersect_flag== True:
                self.get_logger().info('Vuelta a la DERECHA')
                self.next = False
                self.distance = 0.35
                self.time = 3.0


                self.time_extra=1.0
                self.distance_extra = 0.1

                self.flag= False

                if self.state == "stop":
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0
                    self.t_linear = int(self.time * 10**9)
                    self.v_linear = self.distance / self.time

                    self.t_angular = int(self.time * 10**9)
                    self.v_angular = (np.pi/2) / (self.t_angular*10**-9)

                    self.t_extra = int(self.time_extra * 10**9)
                    self.v_extra = self.distance_extra / self.time_extra

                    self.cmd_vel_pub.publish(self.robot_vel)
                    #self.state = "move_right"

                    if self.traffic_received == "verde" or self.traffic_received == "amarillo":
                        self.state = "move_right"
                        self.start_time = self.get_clock().now().nanoseconds
                    
                    elif self.traffic_received == "rojo":
                        self.state = "stop"
                    
                
                elif self.state == "move_right":
                    
                    current_time = self.get_clock().now().nanoseconds
                    if current_time - self.start_time <= self.t_linear:
                        self.robot_vel.linear.x = self.v_linear
                        self.robot_vel.angular.z = 0.0
                        #self.cmd_vel_pub.publish(self.robot_vel)
                    else:
                        if current_time - self.start_time <= self.t_linear + self.t_angular:
                            self.get_logger().info('Vuelta')
                            self.robot_vel.linear.x = 0.0 
                            self.robot_vel.angular.z = -self.v_angular

                            self.cmd_vel_pub.publish(self.robot_vel)      

                        else:
                            if current_time - self.start_time <= self.t_linear + self.t_angular + self.t_extra:
                                self.get_logger().info('Alineando')
                                self.robot_vel.linear.x = self.v_linear
                                self.robot_vel.angular.z = 0.0 
                                
                                self.cmd_vel_pub.publish(self.robot_vel)
                            
                            else:
                                self.start_time = self.get_clock().now().nanoseconds
                                self.intersect_flag = False
                                self.order_received = "Non"
                                self.traffic_received = "NOSEM"
                                self.state = "stop"
                                self.next= True
                    self.cmd_vel_pub.publish(self.robot_vel)
                    

            #######################################################################################################
            #      STOP
            #######################################################################################################
            elif self.order_received == "stop":
                
                self.get_logger().info('Alto por 10 segundos')
                self.next = False
                self.flag = False

                if self.state == "stop":
                    if self.stop_start_time is None:
                        self.stop_start_time = self.get_clock().now().nanoseconds
                    current_time = self.get_clock().now().nanoseconds

                    if current_time - self.stop_start_time <= 10 * 10**9:  # 10 seconds
                        self.robot_vel.linear.x = 0.0
                        self.robot_vel.angular.z = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)
                    else:
                        self.stop_start_time = None
                        self.intersect_flag = False
                        self.order_received = "Non"
                        self.state = "stop"
                        self.next = True



            #######################################################################################################
            #      SEÑAL DE PROGRESS 5
            #######################################################################################################
            elif self.order_received == "progress":

                self.get_logger().info('reduciendo por 5 segundos')
                self.next = False
                self.flag = False

                if self.state == "stop":
                    if self.stop_start_time is None:
                        self.stop_start_time = self.get_clock().now().nanoseconds
                    current_time = self.get_clock().now().nanoseconds

                    if current_time - self.stop_start_time <= 5 * 10**9:  # 10 seconds
                        self.robot_vel.linear.x = 0.05
                        self.get_logger().info('reduciendo por 5 segundos')

                        self.cmd_vel_pub.publish(self.robot_vel)
                    else:
                        self.get_logger().info('BYE reducir')
                        self.stop_start_time = None
                        self.intersect_flag = False
                        self.order_received = "Non"
                        self.state = "stop"
                        self.next = True

            #######################################################################################################
            #      SEÑAL DE GIVEWAY 3
            #######################################################################################################

            elif self.order_received == "giveway":

                self.get_logger().info('reduciendo por 3 segundos')
                self.next = False
                self.flag = False

                if self.state == "stop":
                    if self.stop_start_time is None:
                        self.stop_start_time = self.get_clock().now().nanoseconds
                    current_time = self.get_clock().now().nanoseconds

                    if current_time - self.stop_start_time <= 3 * 10**9:  # 10 seconds
                        self.robot_vel.linear.x = 0.05
                        self.get_logger().info('reduciendo por 3 segundos')

                        self.cmd_vel_pub.publish(self.robot_vel)
                    else:
                        self.get_logger().info('BYE reducir')
                        self.stop_start_time = None
                        self.intersect_flag = False
                        self.order_received = "Non"
                        self.state = "stop"
                        self.next = True
            
                        



            #######################################################################################################
            #      SIN SEÑAL
            #######################################################################################################
            #elif self.order_received == "Non" and not self.intersect_flag:
                #self.get_logger().info('No line detected')
                #self.robot_vel.angular.z = 0.0
                #self.robot_vel.linear.x = 0.0

            self.cmd_vel_pub.publish(self.robot_vel)
            self.pub.publish(self.bridge.cv2_to_imgmsg(intersection_img, 'bgr8'))



def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()