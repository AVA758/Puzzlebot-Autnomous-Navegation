
"""
###DIBUJAAAAA

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from yolo_msg.msg import InferenceResult
from yolo_msg.msg import Yolov8Inference
from std_msgs.msg import String

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Initialize the YOLO model
        self.model = YOLO('~/ros2_ws/src/yolov8_ros2/yolov8_ros2/actual2.pt')
        
        self.yolov8_inference = Yolov8Inference()
        self.bridge = CvBridge()
        
        #PUzzle video_source/raw
        # Subscription to the camera image topic
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)

        self.image_width = 800
        self.image_height = 800
        self.img = np.ndarray((self.image_height, self.image_width, 3))
        
        # Publishers for inference results and image
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

        self.state_pub = self.create_publisher(String, "/sign_order", 1)
        
        # Timer to periodically perform inference
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        results = self.model(self.img)
        
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        detected_objects = []  # Collect detected objects' names

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                class_name = self.model.names[int(c)]
                detected_objects.append(class_name)  # Add detected object's name

        # Publish the names of detected objects as a single string message
        object_names_msg = String()
        object_names_msg.data = ', '.join(detected_objects)

        if object_names_msg.data != '':
            self.state_pub.publish(object_names_msg)

        # Annotate the frame
        annotated_frame = results[0].plot()

        # Rotate the annotated frame back to the original orientation
        annotated_frame = cv2.rotate(annotated_frame, cv2.ROTATE_180)

        img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.img_pub.publish(img_msg)
        
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # Rotate the image to correct the initial upside-down orientation
        # img = cv2.rotate(img, cv2.ROTATE_180)
        
        # Crop the image based on the specified coordinates
        # For example, to crop the region from row 140 to 180 and column 173 to 260

        img = img[20:180, 90:260]
        
        # Resize the cropped image to 800x800 (or other desired size if needed)
        
        self.img = cv2.resize(img, (self.image_width, self.image_height))

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""

###SIN DIBUJAR

"""
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from yolo_msg.msg import InferenceResult
from yolo_msg.msg import Yolov8Inference
from std_msgs.msg import String

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Initialize the YOLO model
        self.model = YOLO('~/ros2_ws/src/yolov8_ros2/yolov8_ros2/actual2.pt')
        
        self.yolov8_inference = Yolov8Inference()
        self.bridge = CvBridge()
        
        # Subscription to the camera image topic
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)

        self.image_width = 800
        self.image_height = 800
        self.img = np.ndarray((self.image_height, self.image_width, 3))
        
        # Publishers for inference results and image
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.state_pub = self.create_publisher(String, "/sign_order", 1)
        
        # Timer to periodically perform inference
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        results = self.model(self.img)
        
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        detected_objects = []  # Collect detected objects' names

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                class_name = self.model.names[int(c)]
                detected_objects.append(class_name)  # Add detected object's name

        # Publish the names of detected objects one at a time
        for detected_object in detected_objects:
            object_name_msg = String()
            object_name_msg.data = detected_object
            self.state_pub.publish(object_name_msg)

        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # Rotate the image to correct the initial upside-down orientation
        img = cv2.rotate(img, cv2.ROTATE_180)
        
        # Crop the image based on the specified coordinates
        # For example, to crop the region from row 140 to 180 and column 173 to 260
        cropped_img = img[20:180, 90:260]
        
        # Resize the cropped image to 800x800 (or other desired size if needed)
        self.img = cv2.resize(cropped_img, (self.image_width, self.image_height))

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""










#counterrrr

"""

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from yolo_msg.msg import InferenceResult
from yolo_msg.msg import Yolov8Inference
from std_msgs.msg import String

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Initialize the YOLO model
        self.model = YOLO('~/ros2_ws/src/yolov8_ros2/yolov8_ros2/actual2.pt')
        
        self.yolov8_inference = Yolov8Inference()
        self.bridge = CvBridge()
        
        # Subscription to the camera image topic
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)

        self.image_width = 800
        self.image_height = 800
        self.img = np.ndarray((self.image_height, self.image_width, 3))
        
        # Publishers for inference results and image
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.state_pub = self.create_publisher(String, "/sign_order", 1)
        
        # Timer to periodically perform inference
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Variables to track consecutive detections
        self.last_detected_object = None
        self.consecutive_count = 0

    def timer_callback(self):

        results = self.model(self.img)
        
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        detected_objects = []  # Collect detected objects' names

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                class_name = self.model.names[int(c)]
                detected_objects.append(class_name)  # Add detected object's name

        # Check for consecutive detections
        if detected_objects:
            most_recent_detection = detected_objects[0]  # Assuming only one object is detected for simplicity
            if most_recent_detection == self.last_detected_object:
                self.consecutive_count += 1
            else:
                self.last_detected_object = most_recent_detection
                self.consecutive_count = 1

            # Publish if the same sign is detected 3 times in a row
            if self.consecutive_count >= 5:
                object_name_msg = String()
                object_name_msg.data = most_recent_detection
                self.state_pub.publish(object_name_msg)
                self.consecutive_count = 0  # Reset the count after publishing
        else:
            self.last_detected_object = None
            self.consecutive_count = 0

        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        # Rotate the image to correct the initial upside-down orientation
        # img = cv2.rotate(img, cv2.ROTATE_180)
        
        # Crop the image based on the specified coordinates
        # For example, to crop the region from row 140 to 180 and column 173 to 260

        #cropped_img = img[20:180, 90:260]
        
        # Resize the cropped image to 800x800 (or other desired size if needed)
        
        self.img = cv2.resize(img, (self.image_width, self.image_height))
        

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
"""
    #######COUNTER CON CONFIDENCE >85%
    
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from yolo_msg.msg import InferenceResult
from yolo_msg.msg import Yolov8Inference
from std_msgs.msg import String


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Initialize the YOLO model
        self.model = YOLO('~/ros2_ws/src/yolov8_ros2/yolov8_ros2/actual2.pt')
        
        self.yolov8_inference = Yolov8Inference()
        self.bridge = CvBridge()
        
        # Subscription to the camera image topic
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)

        self.image_width = 800
        self.image_height = 800
        self.img = np.ndarray((self.image_height, self.image_width, 3))
        
        # Publishers for inference results and image
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.state_pub = self.create_publisher(String, "/sign_order", 1)
        
        # Timer to periodically perform inference
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Variables to track consecutive detections
        self.last_detected_object = None
        self.consecutive_count = 0

    def timer_callback(self):

        results = self.model(self.img)
        
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        detected_objects = []  

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                class_name = self.model.names[int(c)]
                confidence = box.conf  # confidence
                
                if confidence >= 0.70:
                    detected_objects.append((class_name, confidence))  # Add detected object's name and confidence

        # Check for consecutive detections
        if detected_objects:
            most_recent_detection, confidence = detected_objects[0]  # Assuming only one object is detected for simplicity
            if most_recent_detection == self.last_detected_object:
                self.consecutive_count += 1
            else:
                self.last_detected_object = most_recent_detection
                self.consecutive_count = 1

            

            # Publish if the same sign is detected 3 times in a row
            if self.consecutive_count >= 5:
                object_name_msg = String()
                object_name_msg.data = most_recent_detection
                self.state_pub.publish(object_name_msg)
                self.consecutive_count = 0  # Reset the count after publishing
            
            elif self.consecutive_count >= 4 and (detected_objects[0] == "progress" or detected_objects[0] == "giveway"):
                object_name_msg = String()
                object_name_msg.data = most_recent_detection
                self.state_pub.publish(object_name_msg)
                self.consecutive_count = 0  # Reset the count after publishing


        else:
            self.last_detected_object = None
            self.consecutive_count = 0

        

        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        self.img = cv2.resize(img, (self.image_width, self.image_height))
        

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
