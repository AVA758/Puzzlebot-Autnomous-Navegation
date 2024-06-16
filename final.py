# Importación de bibliotecas necesarias
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Clase principal del seguidor de líneas
class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_bueno')
        self.bridge = CvBridge()

        # SUSCRIPTORES
        # Suscriptor para la cámara
        self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10)
        # Suscriptor para las señales
        self.sub2 = self.create_subscription(String, '/sign_order', self.sign_callback, 10)
        # Suscriptor para el color del semáforo
        self.sub3 = self.create_subscription(String, '/traffic_color', self.traffic_callback, 10)

        # PUBLICADORES
        # Publicador para la imagen procesada
        self.pub = self.create_publisher(Image, 'processed_img', 10)
        # Publicador para los comandos de velocidad
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # DECLARACIÓN DE PARÁMETROS
        self.robot_vel = Twist()
        self.image_received_flag = False
        self.intersect_flag = False
        self.traffic_received = "NOSEM"
        self.left_flag = False

        # Intervalo de tiempo para el timer en segundos
        dt = 0.1

        # Creación de un temporizador que llamará a la función `timer_callback` cada `dt` segundos
        self.timer = self.create_timer(dt, self.timer_callback)

        # Mensaje de registro para indicar que el nodo de seguidor de líneas ha comenzado
        self.get_logger().info('Line Follower Node started')

        # Inicialización de banderas y variables de control
        self.flag = False                   # Banderas para el manejo de estado
        self.distance = 0.0                 # Distancia para moverse cuando se recibe una orden
        self.state = "stop"                 # Estado inicial del robot (detenido)
        self.next = True                    # Indica si el robot está listo para la siguiente acción
        self.time = 0.0                     # Tiempo asignado para completar una acción
        self.v = 0.0                        # Velocidad lineal inicial del robot [m/s]



        # Inicialización de tiempos de inicio para el control del movimiento
        self.stop_start_time = None         
        self.progress_start_time = None     

        # Inicialización de órdenes recibidas
        self.order_received = "Non"         # Última orden recibida
        self.last_order_received = "Non"    # Orden anterior recibida para comparación y actualización



    # CALLBACKS

    ## Detección de señalamiento, se verifica que no se recibe más de una vez el mismo en forma secuencial
    def sign_callback(self, msg):
        if msg.data != self.last_order_received:
            self.order_received = msg.data
            self.last_order_received = msg.data

    ## Detección de color de semáforo
    def traffic_callback(self, msg):
        self.traffic_received = msg.data

    ## Importación de video de Puzzlebot
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

            # Región de interés para el seguimiento de la línea
            line_follow = image[161:240, 0:260]
            
            # Imagen para la detección de intersección
            intersection_img = image[120:240, 173:260]
            intersection_img = cv2.cvtColor(intersection_img, cv2.COLOR_BGR2GRAY)
            intersection_img = cv2.cvtColor(intersection_img, cv2.COLOR_GRAY2BGR) 

            # Conversión de la imagen a espacio de color HSV para la detección de contornos
            hsv = cv2.cvtColor(intersection_img, cv2.COLOR_BGR2HSV)
            min_b = np.array([0, 0, 0])
            max_b = np.array([0, 0, 80])
            mask_b = cv2.inRange(hsv, min_b, max_b)

            # Procesamiento de la máscara para mejorar la detección de contornos
            mask_b = cv2.erode(mask_b, None, iterations=2)
            mask_b = cv2.dilate(mask_b, None, iterations=2)
            res_b = cv2.bitwise_and(intersection_img, intersection_img, mask=mask_b)

            # Encontrar contornos en la imagen de intersección
            contours2, hierarchy = cv2.findContours(mask_b.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Conversión a escala de grises de la región de interés
            gr = cv2.cvtColor(line_follow, cv2.COLOR_BGR2GRAY)
            
            # Umbralización para obtener una imagen binaria
            _, thresholded = cv2.threshold(gr, 100, 255, cv2.THRESH_BINARY_INV)

            # Encontrar contornos en la región de interés
            contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Verificar si se detectan contornos de la línea y no hay intersección
            if contours and len(contours2) <= 2 and not self.intersect_flag:
                self.intersect_flag = False
                largest_contour = max(contours, key=cv2.contourArea)
                C = cv2.moments(largest_contour)
                self.flag = False

                # Si hay más de 2 contornos, se detecta una intersección
                if len(contours2) > 2:
                    self.get_logger().info('INTERSECCIÓN dentro')
                    self.robot_vel.angular.z = 0.0
                    self.robot_vel.linear.x = 0.0
                    self.intersect_flag = True
                    self.flag = True 

                # Si se detecta un contorno válido, se calcula el centroide para seguir la línea
                elif C['m00'] != 0:
                    self.get_logger().info('LINEA DETECTADA')
                    cx = int(C['m10'] / C['m00'])
                    cy = int(C['m01'] / C['m00'])
                    cv2.circle(line_follow, (cx, cy), 10, (0, 255, 0), -1)

                    #Se obtiene el error
                    error = cx - image.shape[1] // 2
                    #se calcula con el controlador proporcional la velocidad angular, la velocidad lineal es constante en el line following
                    self.robot_vel.angular.z = -float(error) 
                    self.robot_vel.angular.z = self.robot_vel.angular.z * 0.01
                    self.robot_vel.linear.x = 0.1

                    # Ajuste de velocidad según el color del semáforo
                    if self.traffic_received == "verde":
                        self.get_logger().info('LINEA DETECTADA SEMÁFORO VERDE A LA VISTA')
                        self.robot_vel.linear.x = 0.1

                    #en amarillo va a la mitad de 0.1
                    elif self.traffic_received == "amarillo":
                        self.get_logger().info('LINEA DETECTADA SEMÁFORO AMARILLO A LA VISTA')
                        self.robot_vel.linear.x = 0.05
                else:
                    # Si no se detecta una línea válida, el robot se detiene
                    self.robot_vel.angular.z = 0.0
                    self.robot_vel.linear.x = 0.0

            elif len(contours2) > 2 and not self.flag and self.next:
                # Si se detecta una intersección y no se ha movido aún, el robot se detiene
                self.get_logger().info('INTERSECCIÓN')
                self.robot_vel.angular.z = 0.0
                self.robot_vel.linear.x = 0.0
                self.intersect_flag = True
                self.flag = True

            # MANEJO DE SEÑALES

            #######################################################################################################
            #      SEÑAL DE CONTINÚA AL FRENTE
            #######################################################################################################
            

            if self.order_received == "frente" and self.intersect_flag == True:
                self.get_logger().info('FRENTE')  # muestra en consola
                self.next = False  # Desactiva la posibilidad de realizar otra acción
                self.distance = 0.6  # Distancia a recorrer hacia adelante
                self.time = 4.0  # Tiempo estimado para recorrer la distancia
                self.flag = False  # Bandera para control interno

                if self.state == "stop":
                    # Detener cualquier movimiento previo
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0
                    self.t_linear = int(self.time * 10**9)  # Convertir el tiempo a nanosegundos
                    self.v_linear = self.distance / self.time  # Calcular velocidad lineal requerida
                    self.cmd_vel_pub.publish(self.robot_vel)  # Publicar el comando de velocidad

                    # Comprobar el estado del semáforo para determinar la siguiente acción
                    if self.traffic_received == "verde" or self.traffic_received == "amarillo":
                        self.state = "move_linear"  # Cambiar al estado de movimiento lineal
                        self.start_time = self.get_clock().now().nanoseconds  # Capturar el tiempo actual en nanosegundos
                    elif self.traffic_received == "rojo":
                        self.state = "stop"  # Mantenerse detenido si el semáforo está en rojo
                    
                elif self.state == "move_linear":
                    # Ejecutar movimiento lineal hacia adelante
                    current_time = self.get_clock().now().nanoseconds
                    self.robot_vel.linear.x = self.v_linear  # Establecer la velocidad lineal calculada
                    self.robot_vel.angular.z = 0.0  # No hay movimiento angular porque es moverse de frente
                    self.cmd_vel_pub.publish(self.robot_vel)  # Publicar el comando de velocidad

                    # Verificar si se ha alcanzado el tiempo estimado de movimiento lineal
                    if current_time - self.start_time >= self.t_linear:
                        # Reiniciar las variables y banderas después de completar el movimiento
                        self.start_time = self.get_clock().now().nanoseconds
                        self.intersect_flag = False
                        self.order_received = "Non"  # Restablecer el comando recibido
                        self.traffic_received = "NOSEM"  # Restablecer el estado del semáforo
                        self.state = "stop"  # Volver al estado de detención
                        self.next = True  # Activar el siguiente paso

                

            #######################################################################################################
            #      SEÑAL DE GIRO A LA IZQUIERDA
            #######################################################################################################
            #detecta intersección y recibe left
            elif self.order_received == "left" and self.intersect_flag == True:
                self.left_flag = True  # Activa la bandera de giro a la izquierda
                self.get_logger().info('Vuelta a la izquierda')  # muestra en consola

                self.next = False  # Desactiva el poder realizar otra acción
                self.distance = 0.35  # Distancia a recorrer para el giro a la izquierda
                self.time = 3.0  # Tiempo estimado para completar el giro

                self.time_extra = 1.0  # Tiempo extra para ajuste adicional
                self.distance_extra = 0.1  # Distancia extra para ajuste adicional

                self.flag = False  # Bandera para control interno

                if self.state == "stop":
                    # Detener cualquier movimiento previo
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0

                    # Calcular tiempos y velocidades para el movimiento de giro
                    self.t_linear = int(self.time * 10**9)
                    self.v_linear = self.distance / self.time

                    self.t_angular = int(self.time * 10**9)
                    self.v_angular = (np.pi/2) / (self.t_angular * 10**-9)

                    self.t_extra = int(self.time_extra * 10**9)
                    self.v_extra = self.distance_extra / self.time_extra

                    self.cmd_vel_pub.publish(self.robot_vel)  # Publicar el comando de velocidad

                    # Determinar la acción a seguir según el estado del semáforo
                    if self.traffic_received == "verde" or self.traffic_received == "amarillo":
                        self.state = "move_left"  # Cambiar al estado de movimiento hacia la izquierda
                        self.start_time = self.get_clock().now().nanoseconds  # Capturar el tiempo actual en nanosegundos
                    elif self.traffic_received == "rojo":
                        self.state = "stop"  # Mantenerse detenido si el semáforo está en rojo
                    
                elif self.state == "move_left":
                    # Ejecutar movimiento hacia la izquierda
                    current_time = self.get_clock().now().nanoseconds

                    if current_time - self.start_time <= self.t_linear:
                        # Movimiento lineal inicial hacia adelante
                        self.robot_vel.linear.x = self.v_linear
                        self.robot_vel.angular.z = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)
                    
                    else:
                        if current_time - self.start_time <= self.t_linear + self.t_angular:
                            # Realizar el giro hacia la izquierda
                            self.get_logger().info('Vuelta')
                            self.robot_vel.linear.x = 0.0
                            self.robot_vel.angular.z = self.v_angular
                            self.cmd_vel_pub.publish(self.robot_vel)
                        
                        else:
                            if current_time - self.start_time <= self.t_linear + self.t_angular + self.t_extra:
                                # Alinear el robot después del giro
                                self.get_logger().info('Alineando')
                                self.robot_vel.linear.x = self.v_linear
                                self.robot_vel.angular.z = 0.0
                                self.cmd_vel_pub.publish(self.robot_vel)
                            
                            else:
                                # Reiniciar variables y banderas después de completar la acción
                                self.start_time = self.get_clock().now().nanoseconds
                                self.intersect_flag = False
                                self.order_received = "Non"  # Restablecer el comando recibido
                                self.traffic_received = "NOSEM"  # Restablecer el estado del semáforo
                                self.state = "stop"  # Volver al estado de detención
                                self.next = True  # Activar el siguiente paso
                                self.left_flag = False  # Desactivar la bandera de giro a la izquierda

                    self.cmd_vel_pub.publish(self.robot_vel)  # Publicar el comando de velocidad
                        
              

            
            #######################################################################################################
            #      SEÑAL DE GIRO A LA DERECHA
            #######################################################################################################
            elif self.order_received == "right" and self.intersect_flag == True:
                self.get_logger().info('Vuelta a la DERECHA')  # muestra en consola
                self.next = False  # Desactiva el siguiente paso
                self.distance = 0.35  # Distancia a recorrer para el giro a la derecha
                self.time = 3.0  # Tiempo estimado para completar el giro

                self.time_extra = 1.0  # Tiempo extra para ajuste adicional
                self.distance_extra = 0.1  # Distancia extra para ajuste adicional

                self.flag = False  # Bandera para control interno

                if self.state == "stop":
                    # Detener cualquier movimiento previo
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0

                    # Calcular tiempos y velocidades para el movimiento de giro
                    self.t_linear = int(self.time * 10**9)
                    self.v_linear = self.distance / self.time

                    self.t_angular = int(self.time * 10**9)
                    self.v_angular = (np.pi/2) / (self.t_angular * 10**-9)

                    self.t_extra = int(self.time_extra * 10**9)
                    self.v_extra = self.distance_extra / self.time_extra

                    self.cmd_vel_pub.publish(self.robot_vel)  # Publicar el comando de velocidad

                    # Determinar la acción a seguir según el estado del semáforo
                    if self.traffic_received == "verde" or self.traffic_received == "amarillo":
                        self.state = "move_right"  # Cambiar al estado de movimiento hacia la derecha
                        self.start_time = self.get_clock().now().nanoseconds  # Capturar el tiempo actual en nanosegundos
                    elif self.traffic_received == "rojo":
                        self.state = "stop"  # Mantenerse detenido si el semáforo está en rojo
                    
                elif self.state == "move_right":
                    # Ejecutar movimiento hacia la derecha
                    current_time = self.get_clock().now().nanoseconds

                    if current_time - self.start_time <= self.t_linear:
                        # Movimiento lineal inicial hacia adelante
                        self.robot_vel.linear.x = self.v_linear
                        self.robot_vel.angular.z = 0.0
                        #self.cmd_vel_pub.publish(self.robot_vel)  # Publicar el comando de velocidad
                    
                    else:
                        if current_time - self.start_time <= self.t_linear + self.t_angular:
                            # Realizar el giro hacia la derecha
                            self.get_logger().info('Vuelta')
                            self.robot_vel.linear.x = 0.0
                            self.robot_vel.angular.z = -self.v_angular
                            self.cmd_vel_pub.publish(self.robot_vel)
                        
                        else:
                            if current_time - self.start_time <= self.t_linear + self.t_angular + self.t_extra:
                                # Alinear el robot después del giro
                                self.get_logger().info('Alineando')
                                self.robot_vel.linear.x = self.v_linear
                                self.robot_vel.angular.z = 0.0
                                self.cmd_vel_pub.publish(self.robot_vel)
                            
                            else:
                                # Reiniciar variables y banderas después de completar la acción
                                self.start_time = self.get_clock().now().nanoseconds
                                self.intersect_flag = False
                                self.order_received = "Non"  # Restablecer el comando recibido
                                self.traffic_received = "NOSEM"  # Restablecer el estado del semáforo
                                self.state = "stop"  # Volver al estado de detención
                                self.next = True  # Activar el siguiente paso

                    self.cmd_vel_pub.publish(self.robot_vel)  # Publicar el comando de velocidad

                    

            #######################################################################################################
            #      STOP
            #######################################################################################################
            elif self.order_received == "stop":
                # Caso en que se recibe la orden "stop"
                self.get_logger().info('Alto por 10 segundos')  # Muestra en consola
                self.next = False  # Desactiva el siguiente paso
                self.flag = False  # Bandera para control interno

                if self.state == "stop":
                    if self.stop_start_time is None:
                        self.stop_start_time = self.get_clock().now().nanoseconds
                    current_time = self.get_clock().now().nanoseconds

                    if current_time - self.stop_start_time <= 10 * 10**9:  # 10 segundos en nanosegundos
                        self.robot_vel.linear.x = 0.0
                        self.robot_vel.angular.z = 0.0
                        self.cmd_vel_pub.publish(self.robot_vel)  # Publicar el comando de velocidad
                    else:
                        self.stop_start_time = None  # Reiniciar el tiempo de inicio
                        self.intersect_flag = False
                        self.order_received = "Non"  # Restablecer el comando recibido
                        self.state = "stop"  # Volver al estado de detención
                        self.next = True  # Activar el siguiente paso

            #######################################################################################################
            # SEÑAL DE PROGRESS 5
            #######################################################################################################
            elif self.order_received == "progress":
                # Caso en que se recibe la orden "progress"
                self.get_logger().info('reduciendo por 5 segundos')  # Muestra en consola
                self.next = False  # Desactiva el siguiente paso
                self.flag = False  # Bandera para control interno

                if self.state == "stop":
                    if self.stop_start_time is None:
                        self.stop_start_time = self.get_clock().now().nanoseconds
                    current_time = self.get_clock().now().nanoseconds

                    if current_time - self.stop_start_time <= 5 * 10**9:  # 5 segundos en nanosegundos
                        self.robot_vel.linear.x = 0.05  # Velocidad reducida
                        self.get_logger().info('reduciendo por 5 segundos')
                        self.cmd_vel_pub.publish(self.robot_vel)  # Publicar el comando de velocidad
                    else:
                        self.get_logger().info('BYE reducir')
                        self.stop_start_time = None  # Reiniciar el tiempo de inicio
                        self.intersect_flag = False
                        self.order_received = "Non"  # Restablecer el comando recibido
                        self.state = "stop"  # Volver al estado de detención
                        self.next = True  # Activar el siguiente paso

            #######################################################################################################
            # SEÑAL DE GIVEWAY 3
            #######################################################################################################
            elif self.order_received == "giveway":
                # Caso en que se recibe la orden "giveway"
                self.get_logger().info('reduciendo por 3 segundos')  # Muestra en consola
                self.next = False  # Desactiva el siguiente paso
                self.flag = False  # Bandera para control interno

                if self.state == "stop":
                    if self.stop_start_time is None:
                        self.stop_start_time = self.get_clock().now().nanoseconds
                    current_time = self.get_clock().now().nanoseconds

                    if current_time - self.stop_start_time <= 3 * 10**9:  # 3 segundos en nanosegundos
                        self.robot_vel.linear.x = 0.05  # Velocidad reducida
                        self.get_logger().info('reduciendo por 3 segundos')
                        self.cmd_vel_pub.publish(self.robot_vel)  # Publicar el comando de velocidad
                    else:
                        self.get_logger().info('BYE reducir')
                        self.stop_start_time = None  # Reiniciar el tiempo de inicio
                        self.intersect_flag = False
                        self.order_received = "Non"  # Restablecer el comando recibido
                        self.state = "stop"  # Volver al estado de detención
                        self.next = True  # Activar el siguiente paso


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