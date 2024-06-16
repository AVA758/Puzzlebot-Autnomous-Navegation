# Importa las bibliotecas necesarias
from ultralytics import YOLO  # Importa la clase YOLO desde Ultralytics
import rclpy  # Importa la biblioteca ROS 2 para Python
from rclpy.node import Node  # Importa la clase Node de rclpy
from sensor_msgs.msg import Image  # Importa el mensaje Image de sensor_msgs
from cv_bridge import CvBridge  # Importa CvBridge para convertir entre ROS Image y OpenCV
import cv2  # Importa OpenCV
import numpy as np  # Importa NumPy para operaciones numéricas
from yolo_msg.msg import InferenceResult  # Importa mensajes personalizados de YOLO
from yolo_msg.msg import Yolov8Inference  # Importa mensajes personalizados de YOLO
from std_msgs.msg import String  # Importa el mensaje String de std_msgs

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')  # Inicializa el nodo de ROS con el nombre 'camera_subscriber'
        
        # Inicializa el modelo YOLO con el archivo de pesos especificado
        self.model = YOLO('~/ros2_ws/src/yolov8_ros2/yolov8_ros2/semaforos.pt')
        
        self.yolov8_inference = Yolov8Inference()  # Crea una instancia del mensaje Yolov8Inference
        self.bridge = CvBridge()  # Crea una instancia de CvBridge para conversiones de imagen

        # Suscripción al topic de la imagen de la cámara
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)

        self.image_width = 800  # Ancho deseado de la imagen de entrada
        self.image_height = 800  # Alto deseado de la imagen de entrada
        self.img = np.ndarray((self.image_height, self.image_width, 3))  # Inicializa la matriz de imagen

        # Publicadores para los resultados de inferencia y el estado de la señal
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.state_pub = self.create_publisher(String, "/traffic_color", 1)
        
        # Timer para realizar inferencia periódicamente
        timer_period = 0.2  # segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Método llamado por el temporizador para realizar inferencia y publicar resultados
        
        results = self.model(self.img)  # Realiza la inferencia en la imagen actual
        
        # Configura el encabezado de la inferencia YOLOv8
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        detected_objects = []  # Lista para almacenar nombres de objetos detectados

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # Copia las coordenadas de la caja
                c = box.cls  # Clase del objeto detectado
                class_name = self.model.names[int(c)]  # Nombre de la clase detectada
                detected_objects.append(class_name)  # Agrega el nombre del objeto detectado a la lista

        # Publica los nombres de los objetos detectados uno por uno
        for detected_object in detected_objects:
            object_name_msg = String()
            object_name_msg.data = detected_object

            # Publica solo si el objeto detectado es "rojo", "amarillo" o "verde"
            if object_name_msg.data == "rojo" or object_name_msg.data == "amarillo" or object_name_msg.data == "verde":
                self.state_pub.publish(object_name_msg)

        self.yolov8_pub.publish(self.yolov8_inference)  # Publica los resultados de la inferencia
        self.yolov8_inference.yolov8_inference.clear()  # Limpia los resultados de la inferencia para la siguiente iteración

    def camera_callback(self, data):
        # Método llamado cada vez que se recibe una imagen de la cámara
        
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")  # Convierte el mensaje de imagen a formato OpenCV
        
        # Redimensiona la imagen a las dimensiones deseadas
        self.img = cv2.resize(img, (self.image_width, self.image_height))

def main(args=None):
    rclpy.init(args=args)  # Inicializa ROS
    camera_subscriber = CameraSubscriber()  # Crea una instancia de CameraSubscriber
    rclpy.spin(camera_subscriber)  # Ejecuta el nodo de ROS
    rclpy.shutdown()  # Apaga ROS cuando se cierra el nodo

if __name__ == '__main__':
    main()  # Llama a la función main cuando se ejecuta este script
